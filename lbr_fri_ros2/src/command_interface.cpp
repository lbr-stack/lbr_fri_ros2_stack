#include "lbr_fri_ros2/command_interface.hpp"

namespace lbr_fri_ros2 {

CommandInterface::CommandInterface(const rclcpp::Node::SharedPtr node_ptr)
    : logging_interface_ptr_(node_ptr->get_node_logging_interface()),
      parameters_interface_ptr_(node_ptr->get_node_parameters_interface()) {
  rclcpp::Parameter command_guard_variant_param;
  if (!parameters_interface_ptr_->has_parameter("command_guard_variant")) {
    parameters_interface_ptr_->declare_parameter("command_guard_variant",
                                                 rclcpp::ParameterValue("safe_stop"));
  }
  parameters_interface_ptr_->get_parameter("command_guard_variant", command_guard_variant_param);
  RCLCPP_INFO(logging_interface_ptr_->get_logger(),
              "Configuring command interface with command guard '%s'.",
              command_guard_variant_param.as_string().c_str());
  command_guard_ = command_guard_factory(logging_interface_ptr_, parameters_interface_ptr_,
                                         command_guard_variant_param.as_string());
};

void CommandInterface::get_joint_position_command(fri_command_t_ref command,
                                                  const_fri_state_t_ref state) {
  if (state.getClientCommandMode() != KUKA::FRI::EClientCommandMode::POSITION) {
    std::string err = "Set joint position only allowed in position command mode.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(state.getSampleTime() * 1.0, 0.0, 0.0, 0.0, 0.0, false);
  }
  joint_position_pid_.compute(
      command_target_.joint_position, state.getMeasuredJointPosition(),
      std::chrono::nanoseconds(static_cast<int64_t>(state.getSampleTime() * 1.e9)),
      command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // write joint position to output
  command.setJointPosition(command_.joint_position.data());
}

void CommandInterface::get_torque_command(fri_command_t_ref command, const_fri_state_t_ref state) {
  if (state.getClientCommandMode() != KUKA::FRI::EClientCommandMode::TORQUE) {
    std::string err = "Set torque only allowed in torque command mode.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(state.getSampleTime() * 1.0, 0.0, 0.0, 0.0, 0.0, false);
  }
  joint_position_pid_.compute(
      command_target_.joint_position, state.getMeasuredJointPosition(),
      std::chrono::nanoseconds(static_cast<int64_t>(state.getSampleTime() * 1.e9)),
      command_.joint_position);
  command_.torque = command_target_.torque;

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    throw std::runtime_error("Invalid command.");
  }

  // write joint position and torque to output
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_.torque.data());
}

void CommandInterface::get_wrench_command(fri_command_t_ref command, const_fri_state_t_ref state) {
  if (state.getClientCommandMode() != KUKA::FRI::EClientCommandMode::WRENCH) {
    std::string err = "Set wrench only allowed in wrench command mode.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(state.getSampleTime() * 1.0, 0.0, 0.0, 0.0, 0.0, false);
  }
  joint_position_pid_.compute(
      command_target_.joint_position, state.getMeasuredJointPosition(),
      std::chrono::nanoseconds(static_cast<int64_t>(state.getSampleTime() * 1.e9)),
      command_.joint_position);
  command_.wrench = command_target_.wrench;

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // write joint position and wrench to output
  command.setJointPosition(command_.joint_position.data());
  command.setWrench(command_.wrench.data());
}

void CommandInterface::init_command(const_fri_state_t_ref state) {
  std::memcpy(command_target_.joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  command_ = command_target_;
}
} // namespace lbr_fri_ros2