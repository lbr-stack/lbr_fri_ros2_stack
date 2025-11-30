#include "lbr_ros2_control/controllers/twist_controller.hpp"

namespace lbr_ros2_control {
TwistController::TwistController() : rt_twist_ptr_(nullptr), twist_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
TwistController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
TwistController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SESSION_STATE);
  return interface_configuration;
}

controller_interface::CallbackReturn TwistController::on_init() {
  try {
    twist_subscription_ptr_ = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "command/twist", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          rt_twist_ptr_.writeFromNonRT(msg);
          updates_since_last_command_ = 0;
        });
    this->get_node()->declare_parameter("robot_name", "lbr");
    this->get_node()->declare_parameter("inv_jac_ctrl.chain_root", "lbr_link_0");
    this->get_node()->declare_parameter("inv_jac_ctrl.chain_tip", "lbr_link_ee");
    this->get_node()->declare_parameter("inv_jac_ctrl.twist_in_tip_frame", true);
    this->get_node()->declare_parameter("inv_jac_ctrl.damping", 0.2);
    this->get_node()->declare_parameter("inv_jac_ctrl.max_linear_velocity", 0.1);
    this->get_node()->declare_parameter("inv_jac_ctrl.max_angular_velocity", 0.1);
    this->get_node()->declare_parameter("inv_jac_ctrl.joint_gains",
                                        std::vector<double>(lbr_fri_ros2::N_JNTS, 0.0));
    this->get_node()->declare_parameter("inv_jac_ctrl.cartesian_gains",
                                        std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    this->get_node()->declare_parameter("timeout", 0.2);
    configure_joint_names_();
    configure_joint_limits_();
    configure_inv_jac_ctrl_impl_();
    log_info_();
    timeout_ = this->get_node()->get_parameter("timeout").as_double();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to initialize twist controller with: %s.",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TwistController::update(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration &period) {
  auto twist_command = rt_twist_ptr_.readFromRT();
  if (!twist_command || !(*twist_command)) {
    return controller_interface::return_type::OK;
  }
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

  if (updates_since_last_command_ > static_cast<int>(timeout_ / period.seconds())) {
    zero_joint_velocity_command_();
  } else {
    // compute the joint velocity from the twist command target
    inv_jac_ctrl_impl_ptr_->compute(*twist_command, q_, dq_);
  }

  // pass joint positions to hardware
  auto update_rate = static_cast<double>(get_update_rate());
  if (update_rate <= 0.0) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Update rate should be greater than zero, got %f.",
                 update_rate);
    return controller_interface::return_type::ERROR;
  }
  auto dt = 1. / update_rate;

  // compute new target
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    q_target_[i] = q_[i] + dq_[i] * dt;
  }

  // check target validity and override otherwise
  if (!lbr_fri_ros2::all_jnts_in_bounds(q_target_, lower_joint_limits_, upper_joint_limits_)) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 500 /*ms*/,
        lbr_fri_ros2::ColorScheme::WARNING
            << "Overriding command target to current state since one target beyond joint limits."
            << lbr_fri_ros2::ColorScheme::ENDC);
    q_target_ = q_;
  }

  // set values
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    if (!this->command_interfaces_[i].set_value(q_target_[i])) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Failed to set joint position for joint '" << joint_names_[i]
                              << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    };
  }

  ++updates_since_last_command_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
TwistController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!assign_state_interfaces_()) {
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  reset_command_buffer_();
  zero_joint_velocity_command_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_state_interfaces_();
  reset_command_buffer_();
  zero_joint_velocity_command_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool TwistController::assign_state_interfaces_() {
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

void TwistController::release_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  session_state_interface_ptr_.reset();
}

void TwistController::reset_command_buffer_() {
  rt_twist_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>>(nullptr);
};

void TwistController::zero_joint_velocity_command_() { std::fill(dq_.begin(), dq_.end(), 0.0); }

void TwistController::configure_joint_names_() {
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

void TwistController::configure_joint_limits_() {
  auto hard_joint_limits = get_hard_joint_limits();
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    auto it = hard_joint_limits.find(joint_names_[i]);
    if (it == hard_joint_limits.end()) {
      throw std::runtime_error("Could not find joint limits for '" + joint_names_[i] + "'.");
    }
    lower_joint_limits_[i] = it->second.min_position;
    upper_joint_limits_[i] = it->second.max_position;
  }
}

void TwistController::configure_inv_jac_ctrl_impl_() {
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
          this->get_node()->get_parameter("inv_jac_ctrl.twist_in_tip_frame").as_bool(),
          this->get_node()->get_parameter("inv_jac_ctrl.damping").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_linear_velocity").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_angular_velocity").as_double(),
          joint_gains_array, cartesian_gains_array});
}

void TwistController::log_info_() const { inv_jac_ctrl_impl_ptr_->log_info(); }
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::TwistController, controller_interface::ControllerInterface)
