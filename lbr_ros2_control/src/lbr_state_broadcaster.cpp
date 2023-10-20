#include "lbr_ros2_control/lbr_state_broadcaster.hpp"

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
    state_publisher_ptr_ = this->get_node()->create_publisher<lbr_fri_msgs::msg::LBRState>(
        "~/state", rclcpp::SensorDataQoS());

    rt_state_publisher_ptr_ =
        std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(
            state_publisher_ptr_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR state broadcaster with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LBRStateBroadcaster::update(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {

  // read state from command interfaces

  // publish state from command interfaces

  // rt_state_publisher_ptr_->msg_

  // state

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  rt_state_publisher_ptr_->msg_.client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.commanded_joint_position.fill(
      std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.commanded_torque.fill(std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.control_mode = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.drive_state = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.external_torque.fill(std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.ipo_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.measured_joint_position.fill(
      std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.measured_torque.fill(std::numeric_limits<double>::quiet_NaN());
  rt_state_publisher_ptr_->msg_.overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.safety_state = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.sample_time = std::numeric_limits<double>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.session_state = std::numeric_limits<int8_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
  rt_state_publisher_ptr_->msg_.tracking_performance = std::numeric_limits<double>::quiet_NaN();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}
} // end of namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRStateBroadcaster,
                       controller_interface::ControllerInterface)
