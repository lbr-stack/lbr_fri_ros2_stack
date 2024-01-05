#include "lbr_fri_ros2/async_client.hpp"

namespace lbr_fri_ros2 {
AsyncClient::AsyncClient(const PIDParameters &pid_parameters,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant,
                         const StateInterfaceParameters &state_interface_parameters,
                         const bool &open_loop)
    : command_interface_(pid_parameters, command_guard_parameters, command_guard_variant),
      state_interface_(state_interface_parameters), open_loop_(open_loop) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     ColorScheme::OKBLUE << "Configuring client" << ColorScheme::ENDC);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Command guard variant '" << command_guard_variant.c_str() << "'");
  command_interface_.log_info();
  state_interface_.log_info();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "Open loop '" << (open_loop_ ? "true" : "false") << "'");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     ColorScheme::OKGREEN << "Client configured" << ColorScheme::ENDC);
}

void AsyncClient::onStateChange(KUKA::FRI::ESessionState old_state,
                                KUKA::FRI::ESessionState new_state) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME),
                     "LBR switched from '"
                         << ColorScheme::OKBLUE << ColorScheme::BOLD
                         << EnumMaps::session_state_map(old_state).c_str() << ColorScheme::ENDC
                         << "' to '" << ColorScheme::OKGREEN << ColorScheme::BOLD
                         << EnumMaps::session_state_map(new_state).c_str() << ColorScheme::ENDC
                         << "'. Control mode '" << ColorScheme::OKBLUE << ColorScheme::BOLD
                         << EnumMaps::control_mode_map(robotState().getControlMode())
                         << ColorScheme::ENDC << "'");
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
        "Unsupported command mode '" + std::to_string(robotState().getClientCommandMode()) + "'";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                        ColorScheme::ERROR << err << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }
}
} // end of namespace lbr_fri_ros2
