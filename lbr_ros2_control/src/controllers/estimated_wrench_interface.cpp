#include "lbr_ros2_control/controllers/estimated_wrench_interface.hpp"

namespace lbr_ros2_control {
EstimatedWrenchInterface::EstimatedWrenchInterface() {}

controller_interface::InterfaceConfiguration
EstimatedWrenchInterface::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
EstimatedWrenchInterface::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position and external torque interfaces
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + HW_IF_EXTERNAL_TORQUE);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn EstimatedWrenchInterface::on_init() {
  try {
    get_node()->declare_parameter("robot_name", "lbr");
    get_node()->declare_parameter("wrench_estimator.chain_root", "lbr_link_0");
    get_node()->declare_parameter("wrench_estimator.chain_tip", "lbr_link_ee");
    get_node()->declare_parameter("wrench_estimator.damping", 0.2);
    get_node()->declare_parameter("wrench_estimator.force_x_th", 2.0);
    get_node()->declare_parameter("wrench_estimator.force_y_th", 2.0);
    get_node()->declare_parameter("wrench_estimator.force_z_th", 2.0);
    get_node()->declare_parameter("wrench_estimator.torque_x_th", 0.5);
    get_node()->declare_parameter("wrench_estimator.torque_y_th", 0.5);
    get_node()->declare_parameter("wrench_estimator.torque_z_th", 0.5);
    configure_joint_names_();
    configure_parameters_();
    wrench_estimator_ptr_ = std::make_unique<lbr_fri_ros2::WrenchEstimator>(
        get_robot_description(), wrench_estimator_parameters_);
    log_info_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to initialize estimated wrench interface with: " << e.what()
                            << "." << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EstimatedWrenchInterface::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_X, &wrench_[0]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_Y, &wrench_[1]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_Z, &wrench_[2]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_X, &wrench_[3]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_Y, &wrench_[4]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_Z, &wrench_[5]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EstimatedWrenchInterface::on_export_reference_interfaces() {
  return {};
}

bool EstimatedWrenchInterface::on_set_chained_mode(bool /*chained_mode*/) {
  // broadcasters don't support chaining:
  // https://github.com/ros-controls/ros2_controllers/issues/2052#issuecomment-3616731834
  return true;
}

controller_interface::return_type
EstimatedWrenchInterface::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                            const rclcpp::Duration & /*period*/) {
  return controller_interface::return_type::OK;
}

controller_interface::return_type
EstimatedWrenchInterface::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                    const rclcpp::Duration & /*period*/) {
  // get joint positions and external torques
  if (!read_state_interfaces_()) {
    return controller_interface::return_type::OK;
  }

  // validate read states
  if (!valid_states_()) {
    return controller_interface::return_type::OK;
  }

  // estimate wrench given states
  estimate_wrench_();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
EstimatedWrenchInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EstimatedWrenchInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!assign_state_interfaces_()) {
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  nan_wrench_();
  nan_referenced_states_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EstimatedWrenchInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool EstimatedWrenchInterface::assign_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.push_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_EXTERNAL_TORQUE) {
      external_torque_state_interfaces_.push_back(std::ref(state_interface));
    }
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint position state interfaces '"
                            << joint_position_state_interfaces_.size()
                            << "' does not match the number of joints "
                               "in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (external_torque_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Number of external torque interfaces '"
                                                      << external_torque_state_interfaces_.size()
                                                      << "' does not match the number of joints "
                                                         "in the robot '"
                                                      << lbr_fri_ros2::N_JNTS << "'."
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void EstimatedWrenchInterface::release_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  external_torque_state_interfaces_.clear();
}

void EstimatedWrenchInterface::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint names '" << joint_names_.size()
                            << "' does not match the number of joints in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void EstimatedWrenchInterface::configure_parameters_() {
  wrench_estimator_parameters_.chain_root =
      get_node()->get_parameter("wrench_estimator.chain_root").as_string();
  wrench_estimator_parameters_.chain_tip =
      get_node()->get_parameter("wrench_estimator.chain_tip").as_string();
  wrench_estimator_parameters_.damping =
      get_node()->get_parameter("wrench_estimator.damping").as_double();
  wrench_estimator_parameters_.force_x_th =
      get_node()->get_parameter("wrench_estimator.force_x_th").as_double();
  wrench_estimator_parameters_.force_y_th =
      get_node()->get_parameter("wrench_estimator.force_y_th").as_double();
  wrench_estimator_parameters_.force_z_th =
      get_node()->get_parameter("wrench_estimator.force_z_th").as_double();
  wrench_estimator_parameters_.torque_x_th =
      get_node()->get_parameter("wrench_estimator.torque_x_th").as_double();
  wrench_estimator_parameters_.torque_y_th =
      get_node()->get_parameter("wrench_estimator.torque_y_th").as_double();
  wrench_estimator_parameters_.torque_z_th =
      get_node()->get_parameter("wrench_estimator.torque_z_th").as_double();
  if (!wrench_estimator_parameters_.valid()) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Invalid wrench estimator parameters."
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Invalid wrench estimator parameters.");
  }
}

bool EstimatedWrenchInterface::read_state_interfaces_() {
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    auto joint_position = joint_position_state_interfaces_[i].get().get_optional();
    if (!joint_position.has_value()) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::WARNING
                                                       << "Failed to get joint position for joint '"
                                                       << joint_names_[i] << "'."
                                                       << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    joint_positions_[i] = *joint_position;

    auto external_torque = external_torque_state_interfaces_[i].get().get_optional();
    if (!external_torque.has_value()) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get external torque for joint '" << joint_names_[i]
                             << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    external_torques_[i] = *external_torque;
  }
  return true;
}

bool EstimatedWrenchInterface::valid_states_() const {
  for (size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    if (std::isnan(joint_positions_[i]) || std::isnan(external_torques_[i])) {
      return false;
    }
  }
  return true;
}

void EstimatedWrenchInterface::nan_wrench_() {
  wrench_.fill(std::numeric_limits<double>::quiet_NaN());
}

void EstimatedWrenchInterface::nan_referenced_states_() {
  joint_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  external_torques_.fill(std::numeric_limits<double>::quiet_NaN());
}

void EstimatedWrenchInterface::estimate_wrench_() {
  wrench_estimator_ptr_->set_q(joint_positions_);
  wrench_estimator_ptr_->set_tau_ext(external_torques_);
  wrench_estimator_ptr_->compute();
  wrench_estimator_ptr_->get_f_ext_tf(wrench_);
}

void EstimatedWrenchInterface::log_info_() const { wrench_estimator_ptr_->log_info(); }
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::EstimatedWrenchInterface,
                       controller_interface::ChainableControllerInterface)
