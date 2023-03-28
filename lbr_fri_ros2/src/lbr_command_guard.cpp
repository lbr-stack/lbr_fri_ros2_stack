#include "lbr_fri_ros2/lbr_command_guard.hpp"

namespace lbr_fri_ros2 {
LBRCommandGuard::LBRCommandGuard(const JointArray &min_position, const JointArray &max_position,
                                 const JointArray &max_velocity, const JointArray &max_torque)
    : min_position_(min_position), max_position_(max_position), max_velocity_(max_velocity),
      max_torque_(max_torque) {}

bool LBRCommandGuard::is_valid_command(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                       const lbr_fri_msgs::msg::LBRState &lbr_state) const {
  switch (lbr_state.client_command_mode) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return false;
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command)) {
      return false;
    }
    if (!command_in_velocity_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (is_nan_(lbr_command.wrench.cbegin(), lbr_command.wrench.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command)) {
      return false;
    }
    if (!command_in_velocity_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (is_nan_(lbr_command.torque.cbegin(), lbr_command.torque.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command)) {
      return false;
    }
    if (!command_in_velocity_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  default:
    printf("Invalid EClientCommandMode provided.\n");
    return false;
  }

  return true;
}

bool LBRCommandGuard::is_nan_(const double *begin, const double *end) const {
  return std::find_if(begin, end, [&](const auto &xi) { return std::isnan(xi); }) != end;
}

bool LBRCommandGuard::command_in_position_limits_(
    const lbr_fri_msgs::msg::LBRCommand &lbr_command) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] < min_position_[i] ||
        lbr_command.joint_position[i] > max_position_[i]) {
      printf("Position command not in limits for joint %ld.\n", i);
      return false;
    }
  }
  return true;
}

bool LBRCommandGuard::command_in_velocity_limits_(
    const lbr_fri_msgs::msg::LBRCommand &lbr_command,
    const lbr_fri_msgs::msg::LBRState &lbr_state) const {
  const double &dt = lbr_state.sample_time;
  for (std::size_t i = 0; i < lbr_command.joint_position[i]; ++i) {
    if (std::abs(lbr_command.joint_position[i] - lbr_state.measured_joint_position[i]) / dt >
        max_velocity_[i]) {
      printf("Velocity not in limits for joint %ld.\n", i);
      return false;
    }
  }
  return true;
}

bool LBRCommandGuard::command_in_torque_limits_(
    const lbr_fri_msgs::msg::LBRCommand &lbr_command,
    const lbr_fri_msgs::msg::LBRState &lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
    if (std::abs(lbr_command.torque[i] + lbr_state.external_torque[i]) > max_torque_[i]) {
      printf("Position command not in limits for joint %ld.\n", i);
      return false;
    }
  }
  return true;
}
} // end of namespace lbr_fri_ros2
