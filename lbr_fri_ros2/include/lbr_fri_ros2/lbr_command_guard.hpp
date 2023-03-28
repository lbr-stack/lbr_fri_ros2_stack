#ifndef LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "fri/friClientIf.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
class LBRCommandGuard {
  using JointArray = lbr_fri_msgs::msg::LBRState::_measured_joint_position_type;

public:
  LBRCommandGuard() = delete;
  LBRCommandGuard(const JointArray &min_position, const JointArray &max_position,
                  const JointArray &max_velocity, const JointArray &max_torque);

  bool is_valid_command(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                        const lbr_fri_msgs::msg::LBRState &lbr_state) const;

protected:
  bool is_nan_(const double *begin, const double *end) const;

  bool command_in_position_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command) const;
  bool command_in_velocity_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                   const lbr_fri_msgs::msg::LBRState &lbr_state) const;
  bool command_in_torque_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                 const lbr_fri_msgs::msg::LBRState &lbr_state) const;

  JointArray min_position_, max_position_, max_velocity_, max_torque_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_
