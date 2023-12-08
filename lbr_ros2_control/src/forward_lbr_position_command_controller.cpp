#include "lbr_ros2_control/forward_lbr_position_command_controller.hpp"

namespace lbr_ros2_control {
ForwardLBRPositionCommandController::ForwardLBRPositionCommandController()
    : rt_lbr_position_command_ptr_(nullptr), lbr_position_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
ForwardLBRPositionCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
ForwardLBRPositionCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardLBRPositionCommandController::on_init() {
  try {
    lbr_position_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_msgs::msg::LBRPositionCommand>(
            "command/joint_position", 1,
            [this](const lbr_fri_msgs::msg::LBRPositionCommand::SharedPtr msg) {
              rt_lbr_position_command_ptr_.writeFromNonRT(msg);
            });
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize forward LBR position command controller with: %s.",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
ForwardLBRPositionCommandController::update(const rclcpp::Time & /*time*/,
                                            const rclcpp::Duration & /*period*/) {
  auto lbr_position_command = rt_lbr_position_command_ptr_.readFromRT();
  if (!lbr_position_command || !(*lbr_position_command)) {
    return controller_interface::return_type::OK;
  }
  std::for_each(command_interfaces_.begin(), command_interfaces_.end(),
                [lbr_position_command, idx = 0](auto &command_interface) mutable {
                  command_interface.set_value((*lbr_position_command)->joint_position[idx++]);
                });
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ForwardLBRPositionCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardLBRPositionCommandController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardLBRPositionCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}
} // end of namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::ForwardLBRPositionCommandController,
                       controller_interface::ControllerInterface)
