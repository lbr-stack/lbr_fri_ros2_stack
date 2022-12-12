#ifndef LBR_FRI_ROS2__LBR_HPP_
#define LBR_FRI_ROS2__LBR_HPP_

#include <cmath>
#include <limits>
#include <stdexcept>

#include "fri/friClientIf.h"
#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
struct LBR {
  static constexpr uint8_t JOINT_DOF = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
  static constexpr uint8_t CARTESIAN_DOF = 6;

  LBR();

  // initialize
  bool init_fri_msgs();
  bool init_command();
  bool init_state();
  bool init_limits();

  // validity checks
  bool valid_command();
  bool valid_joint_position_command();
  bool valid_torque_command();
  bool valid_wrench_command();

  // limit checks
  bool command_within_limits(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);
  bool joint_position_command_within_limits(const std::vector<double> &lbr_command);
  bool torque_command_within_limits(const std::vector<double> &lbr_command);
  bool wrench_command_within_limits(const std::vector<double> &lbr_command);

  // command and state
  lbr_fri_msgs::msg::LBRCommand::SharedPtr command;
  lbr_fri_msgs::msg::LBRState::SharedPtr state;

  // limits
  std::vector<double> joint_velocity_command_limit;
  std::vector<double> wrench_command_limit;
  std::vector<double> torque_command_limit;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_HPP_
