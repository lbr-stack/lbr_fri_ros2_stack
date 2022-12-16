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

  // validity checks
  bool valid_command(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);
  bool valid_joint_position_command(const std::vector<double>& joint_position_command);
  bool valid_wrench_command(const std::vector<double>& wrench_command);
  bool valid_torque_command(const std::vector<double>& torque_command);

  bool valid_state();

  // command and state
  lbr_fri_msgs::msg::LBRCommand::SharedPtr command;
  lbr_fri_msgs::msg::LBRState::SharedPtr state;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_HPP_
