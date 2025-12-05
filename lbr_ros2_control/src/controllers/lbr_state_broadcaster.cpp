#include "lbr_ros2_control/controllers/lbr_state_broadcaster.hpp"

namespace lbr_ros2_control {
controller_interface::InterfaceConfiguration
LBRStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LBRStateBroadcaster::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn LBRStateBroadcaster::on_init() {
  try {
    state_publisher_ptr_ =
        this->get_node()->create_publisher<lbr_fri_idl::msg::LBRState>("lbr_state", 1);

    rt_state_publisher_ptr_ =
        std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_idl::msg::LBRState>>(
            state_publisher_ptr_);
    this->get_node()->declare_parameter("robot_name", "lbr");
    configure_joint_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to initialize LBR state broadcaster with: " << e.what()
                            << "." << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LBRStateBroadcaster::update(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {
  for (const auto &state_interface : state_interfaces_) {
    auto state = state_interface.get_optional();
    if (!state.has_value()) {
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get state interface value for '"
                             << state_interface.get_name() << "'."
                             << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::OK;
    }
    state_interface_map_[state_interface.get_prefix_name()][state_interface.get_interface_name()] =
        *state;
  }
  // check any for nan
  if (std::isnan(state_interface_map_[joint_names_[0]][hardware_interface::HW_IF_POSITION])) {
    return controller_interface::return_type::OK;
  }

  // FRI related states
  lbr_state_.client_command_mode =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_CLIENT_COMMAND_MODE]);
  lbr_state_.connection_quality =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_CONNECTION_QUALITY]);
  lbr_state_.control_mode =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_CONTROL_MODE]);
  lbr_state_.drive_state =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_DRIVE_STATE]);
  lbr_state_.operation_mode =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_OPERATION_MODE]);
  lbr_state_.overlay_type =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_OVERLAY_TYPE]);
  lbr_state_.safety_state =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_SAFETY_STATE]);
  lbr_state_.sample_time = state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_SAMPLE_TIME];
  lbr_state_.session_state =
      static_cast<int8_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_SESSION_STATE]);
  lbr_state_.time_stamp_nano_sec = static_cast<uint32_t>(
      state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_TIME_STAMP_NANO_SEC]);
  lbr_state_.time_stamp_sec =
      static_cast<uint32_t>(state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_TIME_STAMP_SEC]);
  lbr_state_.tracking_performance =
      state_interface_map_[HW_IF_AUXILIARY_PREFIX][HW_IF_TRACKING_PERFORMANCE];

  // joint related states
  std::for_each(joint_names_.begin(), joint_names_.end(),
                [&, idx = 0](const std::string &joint_name) mutable {
#if FRI_CLIENT_VERSION_MAJOR == 1
                  lbr_state_.commanded_joint_position[idx] =
                      state_interface_map_[joint_name][HW_IF_COMMANDED_JOINT_POSITION];
#endif
                  lbr_state_.commanded_torque[idx] =
                      state_interface_map_[joint_name][HW_IF_COMMANDED_TORQUE];
                  lbr_state_.external_torque[idx] =
                      state_interface_map_[joint_name][HW_IF_EXTERNAL_TORQUE];
                  if (lbr_state_.session_state == KUKA::FRI::COMMANDING_WAIT ||
                      lbr_state_.session_state == KUKA::FRI::COMMANDING_ACTIVE) {
                    lbr_state_.ipo_joint_position[idx] =
                        state_interface_map_[joint_name][HW_IF_IPO_JOINT_POSITION];
                  } else {
                    lbr_state_.ipo_joint_position[idx] = std::numeric_limits<double>::quiet_NaN();
                  }
                  lbr_state_.measured_joint_position[idx] =
                      state_interface_map_[joint_name][hardware_interface::HW_IF_POSITION];
                  lbr_state_.measured_torque[idx] =
                      state_interface_map_[joint_name][hardware_interface::HW_IF_EFFORT];
                  ++idx;
                });
  rt_state_publisher_ptr_->try_publish(lbr_state_);
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  init_state_interface_map_();
  init_state_msg_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void LBRStateBroadcaster::init_state_interface_map_() {
  for (const auto &state_interface : state_interfaces_) {
    state_interface_map_[state_interface.get_prefix_name()][state_interface.get_interface_name()] =
        std::numeric_limits<double>::quiet_NaN();
  }
}

void LBRStateBroadcaster::init_state_msg_() {
  lbr_state_.client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
#if FRI_CLIENT_VERSION_MAJOR == 1
  lbr_state_.commanded_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
#endif
  lbr_state_.commanded_torque.fill(std::numeric_limits<double>::quiet_NaN());
  lbr_state_.connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.control_mode = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.drive_state = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.external_torque.fill(std::numeric_limits<double>::quiet_NaN());
  lbr_state_.ipo_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  lbr_state_.measured_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  lbr_state_.measured_torque.fill(std::numeric_limits<double>::quiet_NaN());
  lbr_state_.overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.safety_state = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.sample_time = std::numeric_limits<double>::quiet_NaN();
  lbr_state_.session_state = std::numeric_limits<int8_t>::quiet_NaN();
  lbr_state_.time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
  lbr_state_.time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
  lbr_state_.tracking_performance = std::numeric_limits<double>::quiet_NaN();
}

void LBRStateBroadcaster::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint names '" << joint_names_.size()
                            << "' does not match the number of joints in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRStateBroadcaster,
                       controller_interface::ControllerInterface)
