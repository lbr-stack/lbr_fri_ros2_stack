#include "lbr_fri_ros2/async_client.hpp"

namespace lbr_fri_ros2 {
AsyncClient::AsyncClient(const PIDParameters &pid_parameters,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant,
                         const StateInterfaceParameters &state_interface_parameters,
                         const bool &open_loop)
    : command_interface_(pid_parameters, command_guard_parameters, command_guard_variant),
      state_interface_(state_interface_parameters), open_loop_(open_loop) {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Configuring client.");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Command guard variant: '%s'.",
              command_guard_variant.c_str());
  command_interface_.log_info();
  state_interface_.log_info();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Open loop: '%s'.", open_loop_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Client configured.");
}

void AsyncClient::onStateChange(KUKA::FRI::ESessionState old_state,
                                KUKA::FRI::ESessionState new_state) {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "LBR switched from '%s' to '%s'.",
              EnumMaps::session_state_map(old_state).c_str(),
              EnumMaps::session_state_map(new_state).c_str());
  command_interface_.init_command(robotState());
}

void AsyncClient::monitor() { state_interface_.set_state(robotState()); };

void AsyncClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  state_interface_.set_state(robotState());

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
    command_interface_.get_torque_command(robotCommand(), robotState());
  }

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::WRENCH) {
    command_interface_.get_wrench_command(robotCommand(), robotState());
  }
}

void AsyncClient::command() {
  if (open_loop_) {
    state_interface_.set_state_open_loop(robotState(),
                                         command_interface_.get_command().joint_position);
  } else {
    state_interface_.set_state(robotState());
  }

  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::POSITION:
    command_interface_.get_joint_position_command(robotCommand(), robotState());
    return;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    command_interface_.get_torque_command(robotCommand(), robotState());
    return;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    command_interface_.get_wrench_command(robotCommand(), robotState());
    return;
  default:
    std::string err =
        "Unsupported command mode: " + std::to_string(robotState().getClientCommandMode()) + ".";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
}
} // end of namespace lbr_fri_ros2
