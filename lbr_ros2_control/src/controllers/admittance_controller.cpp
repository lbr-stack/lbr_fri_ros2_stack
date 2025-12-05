#include "lbr_ros2_control/controllers/admittance_controller.hpp"

namespace lbr_ros2_control {
AdmittanceController::AdmittanceController() {}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position interface
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  // estimated force-torque sensor interface
  for (const auto &interface_name : estimated_ft_sensor_ptr_->get_state_interface_names()) {
    interface_configuration.names.push_back(interface_name);
  }

  // additional state interfaces
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SESSION_STATE);
  return interface_configuration;
}

controller_interface::CallbackReturn AdmittanceController::on_init() {
  try {
    this->get_node()->declare_parameter("robot_name", "lbr");
    this->get_node()->declare_parameter("ft_sensor_name", "estimated_wrench_interface");
    this->get_node()->declare_parameter("admittance.mass",
                                        std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 1.0));
    this->get_node()->declare_parameter("admittance.damping",
                                        std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    this->get_node()->declare_parameter("admittance.stiffness",
                                        std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    this->get_node()->declare_parameter("inv_jac_ctrl.chain_root", "lbr_link_0");
    this->get_node()->declare_parameter("inv_jac_ctrl.chain_tip", "lbr_link_ee");
    this->get_node()->declare_parameter("inv_jac_ctrl.damping", 0.2);
    this->get_node()->declare_parameter("inv_jac_ctrl.max_linear_velocity", 0.1);
    this->get_node()->declare_parameter("inv_jac_ctrl.max_angular_velocity", 0.1);
    this->get_node()->declare_parameter("inv_jac_ctrl.joint_gains",
                                        std::vector<double>(lbr_fri_ros2::N_JNTS, 0.0));
    this->get_node()->declare_parameter("inv_jac_ctrl.cartesian_gains",
                                        std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    this->get_node()->declare_parameter("filter.f_ext_tau", 0.4);
    this->get_node()->declare_parameter("on_activate.max_external_force", 0.0);
    this->get_node()->declare_parameter("on_activate.max_external_torque", 0.0);
    configure_joint_names_();
    configure_admittance_impl_();
    configure_inv_jac_ctrl_impl_();
    configure_filters_();
    configure_safety_checks_();
    log_info_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize admittance controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
AdmittanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // get estimated force-torque sensor values
  f_ext_.head(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_forces().data());
  f_ext_.tail(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_torques().data());

  // get joint positions
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    auto q_i = this->joint_position_state_interfaces_[i].get().get_optional();
    if (!q_i.has_value()) {
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get joint position for joint '" << joint_names_[i]
                             << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::OK;
    }
    q_[i] = *q_i;
  }

  // compute forward kinematics
  auto chain_tip_frame = inv_jac_ctrl_impl_ptr_->get_kinematics_ptr()->compute_fk(q_);
  t_ = Eigen::Map<Eigen::Matrix<double, 3, 1>>(chain_tip_frame.p.data);
  r_ = Eigen::Quaterniond(chain_tip_frame.M.data);

  // compute steady state position and orientation
  if (!initialized_) {
    t_init_ = t_;
    t_prev_ = t_init_;
    r_init_ = r_;
    r_prev_ = r_init_;
    initialized_ = true;
  }

  // compute translational delta and velocity
  auto update_rate = static_cast<double>(get_update_rate());
  if (update_rate <= 0.0) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Update rate should be greater than zero, got %f.",
                 update_rate);
    return controller_interface::return_type::ERROR;
  }
  auto dt = 1. / update_rate;
  delta_x_.head(3) = (t_ - t_init_);
  dx_.head(3) = (t_ - t_prev_) / dt;

  // compute rotational delta and veloctity
  Eigen::AngleAxisd deltaa(r_.inverse() * r_init_);
  delta_x_.tail(4) = deltaa.axis() * deltaa.angle();
  Eigen::AngleAxisd da(r_.inverse() * r_prev_);
  dx_.tail(3) = da.axis() * da.angle();

  // update previous values
  t_prev_ = t_;
  r_prev_ = r_;

  // convert f_ext_ back to root frame
  f_ext_.head(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.head(3);
  f_ext_.tail(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.tail(3);

  // filter the external forces
  f_ext_filter_ptr_->compute(f_ext_.data(), f_ext_filtered_.data());

  // compute admittance
  admittance_impl_ptr_->compute(f_ext_filtered_, delta_x_, dx_, ddx_);

  // integrate ddx_ to command velocity
  dx_ += ddx_ * dt;
  twist_command_ = dx_;

  if (!inv_jac_ctrl_impl_ptr_) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inverse Jacobian controller not initialized.");
    return controller_interface::return_type::ERROR;
  }

  // check for robot session state
  auto session_state = session_state_interface_ptr_->get().get_optional();
  if (!session_state.has_value()) {
    RCLCPP_WARN_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::WARNING
                                                           << "Failed to get session state."
                                                           << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::OK;
  }
  if (static_cast<int>(*session_state) != KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    return controller_interface::return_type::OK;
  }

  // compute the joint velocity from the twist command target
  inv_jac_ctrl_impl_ptr_->compute(twist_command_, q_, dq_);

  // pass joint positions to hardware
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    if (!this->command_interfaces_[i].set_value(q_[i] + dq_[i] * dt)) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Failed to set joint position for joint '" << joint_names_[i]
                              << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    };
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
AdmittanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  ft_sensor_name_ = this->get_node()->get_parameter("ft_sensor_name").as_string();
  estimated_ft_sensor_ptr_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      ft_sensor_name_ + "/" + HW_IF_FORCE_X, ft_sensor_name_ + "/" + HW_IF_FORCE_Y,
      ft_sensor_name_ + "/" + HW_IF_FORCE_Z, ft_sensor_name_ + "/" + HW_IF_TORQUE_X,
      ft_sensor_name_ + "/" + HW_IF_TORQUE_Y, ft_sensor_name_ + "/" + HW_IF_TORQUE_Z);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  initialized_ = false;
  if (!assign_state_interfaces_()) {
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  init_filters_with_update_rate_();
  zero_all_values_();
  try {
    if (any_external_force_torques_on_horizon_(max_external_force_on_activate_,
                                               max_external_torque_on_activate_)) {
      RCLCPP_ERROR_STREAM(
          this->get_node()->get_logger(),
          lbr_fri_ros2::ColorScheme::ERROR
              << "External force-torques detected during admittance controller activation. "
                 "Please make sure load data was calibrated."
              << lbr_fri_ros2::ColorScheme::ENDC);
      release_state_interfaces_();
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        this->get_node()->get_logger(),
        lbr_fri_ros2::ColorScheme::ERROR
            << "Failed to check external force-torques during admittance controller activation "
               "with: "
            << e.what() << lbr_fri_ros2::ColorScheme::ENDC);
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AdmittanceController::assign_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.push_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SESSION_STATE) {
      session_state_interface_ptr_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
  }
  if (!estimated_ft_sensor_ptr_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to assign estimated force torque state interfaces.");
    return false;
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  return true;
}

void AdmittanceController::release_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  session_state_interface_ptr_.reset();
  estimated_ft_sensor_ptr_->release_interfaces();
}

void AdmittanceController::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint names '%ld' does not match the number of joints in the robot '%d'.",
        joint_names_.size(), lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void AdmittanceController::configure_admittance_impl_() {
  if (this->get_node()->get_parameter("admittance.mass").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of mass values '%ld' does not match the number of cartesian degrees of "
                 "freedom '%d'.",
                 this->get_node()->get_parameter("admittance.mass").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.damping").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of damping values '%ld' does not match the number of cartesian degrees of freedom "
        "'%d'.",
        this->get_node()->get_parameter("admittance.damping").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.stiffness").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of stiffness values '%ld' does not match the number of cartesian degrees "
                 "of freedom "
                 "'%d'.",
                 this->get_node()->get_parameter("admittance.stiffness").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  lbr_fri_ros2::cart_array_t mass_array;
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    mass_array[i] = this->get_node()->get_parameter("admittance.mass").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t damping_array;
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    damping_array[i] = this->get_node()->get_parameter("admittance.damping").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t stiffness_array;
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    stiffness_array[i] =
        this->get_node()->get_parameter("admittance.stiffness").as_double_array()[i];
  }
  admittance_impl_ptr_ = std::make_unique<lbr_fri_ros2::AdmittanceImpl>(
      lbr_fri_ros2::AdmittanceParameters{mass_array, damping_array, stiffness_array});
}

void AdmittanceController::configure_inv_jac_ctrl_impl_() {
  if (this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size() !=
      lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint gains '%ld' does not match the number of joints in the robot '%d'.",
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size(),
        lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint gains.");
  }
  if (this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of cartesian gains '%ld' does not match the number of cartesian degrees of freedom "
        "'%d'.",
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure cartesian gains.");
  }
  lbr_fri_ros2::jnt_array_t joint_gains_array;
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t cartesian_gains_array;
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    cartesian_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array()[i];
  }
  inv_jac_ctrl_impl_ptr_ = std::make_unique<lbr_fri_ros2::InvJacCtrlImpl>(
      this->get_robot_description(),
      lbr_fri_ros2::InvJacCtrlParameters{
          this->get_node()->get_parameter("inv_jac_ctrl.chain_root").as_string(),
          this->get_node()->get_parameter("inv_jac_ctrl.chain_tip").as_string(),
          false, // always assume twist in root frame
          this->get_node()->get_parameter("inv_jac_ctrl.damping").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_linear_velocity").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_angular_velocity").as_double(),
          joint_gains_array, cartesian_gains_array});
}

void AdmittanceController::configure_filters_() {
  auto f_ext_tau = this->get_node()->get_parameter("filter.f_ext_tau").as_double();
  if (f_ext_tau < 0.2) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "External force filter time constant too small (" << f_ext_tau
                            << "s). Currently enforced to be at least 0.2s for proper smoothing."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Invalid external force filter time constant.");
  }
  f_ext_filter_ptr_ =
      std::make_unique<lbr_fri_ros2::ExponentialFilterArray<lbr_fri_ros2::CARTESIAN_DOF>>(
          f_ext_tau);
}

void AdmittanceController::configure_safety_checks_() {
  max_external_force_on_activate_ =
      this->get_node()->get_parameter("on_activate.max_external_force").as_double();
  max_external_torque_on_activate_ =
      this->get_node()->get_parameter("on_activate.max_external_torque").as_double();
  if (max_external_force_on_activate_ < 0.0) {
    RCLCPP_WARN_STREAM(
        this->get_node()->get_logger(),
        lbr_fri_ros2::ColorScheme::WARNING
            << "Parameter 'on_activate.max_external_force' is negative, overriding to 0.0."
            << lbr_fri_ros2::ColorScheme::ENDC);
    max_external_force_on_activate_ = 0.0;
  }
  if (max_external_torque_on_activate_ < 0.0) {
    RCLCPP_WARN_STREAM(
        this->get_node()->get_logger(),
        lbr_fri_ros2::ColorScheme::WARNING
            << "Parameter 'on_activate.max_external_torque' is negative, overriding to 0.0."
            << lbr_fri_ros2::ColorScheme::ENDC);
    max_external_torque_on_activate_ = 0.0;
  }
}

void AdmittanceController::init_filters_with_update_rate_() {
  f_ext_filter_ptr_->initialize(1. / static_cast<double>(this->get_update_rate()));
}

void AdmittanceController::zero_all_values_() {
  f_ext_.setZero();
  f_ext_filtered_.setZero();
  delta_x_.setZero();
  dx_.setZero();
  ddx_.setZero();
  std::fill(dq_.begin(), dq_.end(), 0.0);
  twist_command_.setZero();
}

bool AdmittanceController::any_external_force_torques_on_horizon_(
    const double &max_external_force, const double &max_external_torque,
    const std::chrono::milliseconds &horizon) const {
  if (max_external_force < 0.0 || max_external_torque < 0.0) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Maximum external force-torque limits must be non-negative."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Invalid maximum external force-torque limits.");
  }
  if (!estimated_ft_sensor_ptr_) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Estimated force-torque sensor not initialized for external force-torque check.");
    throw std::runtime_error("Estimated force-torque sensor not initialized.");
  }
  if (horizon.count() < 100) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Horizon must at least be 100 ms."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Invalid horizon for external force-torque check.");
  }
  auto forces = this->estimated_ft_sensor_ptr_->get_forces();
  auto torques = this->estimated_ft_sensor_ptr_->get_torques();
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < horizon) {
    if (!(lbr_fri_ros2::norm_in_bounds(forces, max_external_force) &&
          lbr_fri_ros2::norm_in_bounds(torques, max_external_torque))) {
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(),
                         "External force-torques detected: forces = ["
                             << forces[0] << ", " << forces[1] << ", " << forces[2]
                             << "], torques = [" << torques[0] << ", " << torques[1] << ", "
                             << torques[2] << "].");
      return true;
    }
    forces = this->estimated_ft_sensor_ptr_->get_forces();
    torques = this->estimated_ft_sensor_ptr_->get_torques();
  }
  return false;
}

void AdmittanceController::log_info_() const {
  admittance_impl_ptr_->log_info();
  inv_jac_ctrl_impl_ptr_->log_info();
  f_ext_filter_ptr_->log_info();
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::AdmittanceController,
                       controller_interface::ControllerInterface)