#include "lbr_fri_ros2/interfaces/torque_only_command.hpp"

namespace lbr_fri_ros2 {
TorqueOnlyCommandInterface::TorqueOnlyCommandInterface(
    const double &joint_position_tau, const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(joint_position_tau, command_guard_parameters, command_guard_variant) {
  init_state = 1;
}

void TorqueOnlyCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                     const_idl_state_t_ref state) {
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::TORQUE) {
    std::string err = "Expected robot in '" +
                      EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::TORQUE) +
                      "' command mode got '" +
                      EnumMaps::client_command_mode_map(state.client_command_mode) + "'";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!command_initialized_) {
    std::string err = "Uninitialized command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!std::any_of(command_target_.torque.cbegin(), command_target_.torque.cend(),
                   [](const double &v) { return std::isnan(v); })) {
    // write command_target_ to command_ else use internal command_
    command_.torque = command_target_.torque;
  }

  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // the current (unchanged) joint position (robot interface needs to send positions for safety
  // measures even when they are not used internally for control)
  command_.joint_position = state.measured_joint_position;

  if (init_state) {
    // no commands received yet - the torque is computed as a simple
    // P-controller for joint positions to avoid robot falling down

    if (init_state == 1) {
      // the first run - save the joints position
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()), ColorScheme::BOLD << ColorScheme::OKBLUE
        << "Init P-hold" << ColorScheme::ENDC);

      for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i ++)
        init_pos[i] = state.measured_joint_position[i];
      init_state = 2;
    }

    double torque[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i ++) {
      if (command_.torque[i] != 0.0) {
        // an input received, end the init state
        init_state = 0;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()), ColorScheme::BOLD << ColorScheme::OKBLUE
          << "Accepting torque control" << ColorScheme::ENDC);
        break;
      }
      // TODO P stiffness is hardcoded here
      torque[i] = 200.0 * (init_pos[i] - state.measured_joint_position[i]);
    }

    if (init_state)
      for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i ++)
          command_.torque[i] = torque[i];
  }

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // write joint position and torque to output
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_.torque.data());
}
} // namespace lbr_fri_ros2
