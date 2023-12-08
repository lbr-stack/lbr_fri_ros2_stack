#include "lbr_fri_ros2/command_interface.hpp"

namespace lbr_fri_ros2 {

CommandInterface::CommandInterface(const PIDParameters &pid_parameters,
                                   const CommandGuardParameters &command_guard_parameters,
                                   const std::string &command_guard_variant)
    : pid_parameters_(pid_parameters) {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
};

void CommandInterface::get_joint_position_command(fri_command_t_ref command,
                                                  const_fri_state_t_ref state) {
  if (state.getClientCommandMode() != KUKA::FRI::EClientCommandMode::POSITION) {
    std::string err = "Set joint position only allowed in position command mode.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(pid_parameters_, state.getSampleTime());
  }
  joint_position_pid_.compute(
      command_target_.joint_position, state.getMeasuredJointPosition(),
      std::chrono::nanoseconds(static_cast<int64_t>(state.getSampleTime() * 1.e9)),
      command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }

  // write joint position to output
  command.setJointPosition(command_.joint_position.data());
}

void CommandInterface::get_torque_command(fri_command_t_ref command, const_fri_state_t_ref state) {
  if (state.getClientCommandMode() != KUKA::FRI::EClientCommandMode::TORQUE) {
    std::string err = "Set torque only allowed in torque command mode.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(pid_parameters_, state.getSampleTime());
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
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }

  // PID
  if (!joint_position_pid_.is_initialized()) {
    joint_position_pid_.initialize(pid_parameters_, state.getSampleTime());
  }
  joint_position_pid_.compute(
      command_target_.joint_position, state.getMeasuredJointPosition(),
      std::chrono::nanoseconds(static_cast<int64_t>(state.getSampleTime() * 1.e9)),
      command_.joint_position);
  command_.wrench = command_target_.wrench;

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
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

void CommandInterface::log_info() const {
  command_guard_->log_info();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.p: %.1f", pid_parameters_.p);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.i: %.1f", pid_parameters_.i);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.d: %.1f", pid_parameters_.d);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.i_max: %.1f", pid_parameters_.i_max);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.i_min: %.1f", pid_parameters_.i_min);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   pid.antiwindup: %s",
              pid_parameters_.antiwindup ? "true" : "false");
}
} // namespace lbr_fri_ros2
