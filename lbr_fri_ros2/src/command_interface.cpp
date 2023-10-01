#include "lbr_fri_ros2/command_interface.hpp"

namespace lbr_fri_ros2 {

CommandInterface::CommandInterface(const rclcpp::Node::SharedPtr node_ptr)
    : logging_interface_ptr_(node_ptr->get_node_logging_interface()),
      parameters_interface_ptr_(node_ptr->get_node_parameters_interface()),
      joint_position_pid_(node_ptr, {"A1", "A2", "A3", "A4", "A5", "A6", "A7"}) {
  command_guard_ = std::make_unique<CommandGuard>(node_ptr->get_node_logging_interface());
  joint_position_pid_.init(0.2, 0.0, 0.0, 0.0, 0.0, false);
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
  joint_position_pid_.compute(command_target_.joint_position, state.getMeasuredJointPosition(),
                              rclcpp::Duration(std::chrono::milliseconds(
                                  static_cast<int64_t>(state.getSampleTime() * 1e3))),
                              command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // write to output
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
  joint_position_pid_.compute(command_target_.joint_position, state.getMeasuredJointPosition(),
                              rclcpp::Duration(std::chrono::milliseconds(
                                  static_cast<int64_t>(state.getSampleTime() * 1e3))),
                              command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    throw std::runtime_error("Invalid command.");
  }

  // write to output
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_target_.torque.data());
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
  joint_position_pid_.compute(command_target_.joint_position, state.getMeasuredJointPosition(),
                              rclcpp::Duration(std::chrono::milliseconds(
                                  static_cast<int64_t>(state.getSampleTime() * 1e3))),
                              command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  // write to output
  command.setJointPosition(state.getMeasuredJointPosition());
  command.setWrench(command_target_.wrench.data());
}

void CommandInterface::init_command(const_fri_state_t_ref state) {
  std::memcpy(command_target_.joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  command_ = command_target_;
}
} // namespace lbr_fri_ros2