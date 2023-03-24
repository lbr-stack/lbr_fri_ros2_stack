#include "lbr_fri_ros2/lbr_limit_guard.hpp"

namespace lbr_fri_ros2 {
LBRJointLimitGuard::LBRJointLimitGuard(const JointArray &min_position,
                                       const JointArray &max_position,
                                       const JointArray &max_velocity, const JointArray &max_torque)
    : min_position_(min_position), max_position_(max_position), max_velocity_(max_velocity),
      max_torque_(max_torque) {}

bool LBRJointLimitGuard::command_in_position_limits(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command) const {
  if (!lbr_command) {
    return false;
  }
  for (std::size_t i = 0; i < lbr_command->joint_position.size(); ++i) {
    if (lbr_command->joint_position[i] < min_position_[i] ||
        lbr_command->joint_position[i] > max_position_[i])
      return false;
  }
  return true;
}

bool LBRJointLimitGuard::command_in_velocity_limits(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command,
    const lbr_fri_msgs::msg::LBRState::ConstSharedPtr &lbr_state) const {
  const double &dt = lbr_state->sample_time;
  if (!lbr_command || !lbr_state) {
    return false;
  }
  for (std::size_t i = 0; i < lbr_command->joint_position[i]; ++i) {
    if (std::abs(lbr_command->joint_position[i] - lbr_state->measured_joint_position[i]) / dt >
        max_velocity_[i])
      return false;
  }
  return true;
}

bool LBRJointLimitGuard::command_in_torque_limits(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command,
    const lbr_fri_msgs::msg::LBRState::ConstSharedPtr &lbr_state) const {
  if (!lbr_command || !lbr_state) {
    return false;
  }
  for (std::size_t i = 0; i < lbr_command->torque.size(); ++i) {
    if (std::abs(lbr_command->torque[i] + lbr_state->external_torque[i]) > max_torque_[i])
      return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
