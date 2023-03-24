#ifndef LBR_FRI_ROS2__LBR_LIMIT_GUARD_HPP_
#define LBR_FRI_ROS2__LBR_LIMIT_GUARD_HPP_

#include <cmath>

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
class LBRJointLimitGuard {
  using JointArray = decltype(lbr_fri_msgs::msg::LBRCommand::joint_position);

public:
  LBRJointLimitGuard() = delete;
  LBRJointLimitGuard(const JointArray &min_position, const JointArray &max_position,
                     const JointArray &max_velocity, const JointArray &max_torque);

  bool command_in_position_limits(
      const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command) const;
  bool
  command_in_velocity_limits(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command,
                             const lbr_fri_msgs::msg::LBRState::ConstSharedPtr &lbr_state) const;
  bool command_in_torque_limits(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr &lbr_command,
                                const lbr_fri_msgs::msg::LBRState::ConstSharedPtr &lbr_state) const;

protected:
  JointArray min_position_, max_position_, max_velocity_, max_torque_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_LIMIT_GUARD_HPP_
