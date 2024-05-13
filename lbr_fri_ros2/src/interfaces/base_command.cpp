#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {

BaseCommandInterface::BaseCommandInterface(const PIDParameters &pid_parameters,
                                           const CommandGuardParameters &command_guard_parameters,
                                           const std::string &command_guard_variant)
    : pid_parameters_(pid_parameters) {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
};

void BaseCommandInterface::init_command(const_idl_state_t_ref state) {
  command_target_.joint_position = state.measured_joint_position;
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  command_ = command_target_;
}

void BaseCommandInterface::log_info() const {
  command_guard_->log_info();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.p: %.1f", pid_parameters_.p);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.i: %.1f", pid_parameters_.i);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.d: %.1f", pid_parameters_.d);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.i_max: %.1f", pid_parameters_.i_max);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.i_min: %.1f", pid_parameters_.i_min);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "*   pid.antiwindup: %s",
              pid_parameters_.antiwindup ? "true" : "false");
}
} // namespace lbr_fri_ros2
