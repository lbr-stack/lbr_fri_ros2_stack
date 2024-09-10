#include "lbr_ros2_control/controllers/admittance_controller.hpp"

namespace lbr_ros2_control {
AdmittanceController::AdmittanceController() {}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() {
  // reference joint position command interface
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() {
  // retrieve estimated ft state interface
}

controller_interface::CallbackReturn AdmittanceController::on_init() {}

controller_interface::return_type AdmittanceController::update(const rclcpp::Time &time,
                                                               const rclcpp::Duration &period) {
  // compute admittance
  // add warning for high force-torques and refer to load data calibration
}

controller_interface::CallbackReturn
AdmittanceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  clear_command_interfaces_();
  clear_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AdmittanceController::reference_command_interfaces_() {
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_command_interfaces_.emplace_back(std::ref(command_interface));
    }
  }
  if (joint_position_command_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position command interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_command_interfaces_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
}

bool AdmittanceController::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == HW_IF_ESTIMATED_FT_PREFIX) {
      estimated_ft_sensor_state_interface_.emplace_back(std::ref(state_interface));
    }
  }
}

void AdmittanceController::clear_command_interfaces_() {
  joint_position_command_interfaces_.clear();
}

void AdmittanceController::clear_state_interfaces_() {
  estimated_ft_sensor_state_interface_.clear();
}
} // namespace lbr_ros2_control
