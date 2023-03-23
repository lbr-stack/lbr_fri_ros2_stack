#ifndef LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
#define LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fri/FRIMessages.pb.h"
#include "fri/friClientIf.h"
#include "fri/friLBRCommand.h"
#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {

class LBRIntermediary {
  static const uint8_t JOINT_DOF = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
  static const uint8_t CARTESIAN_DOF = 6;

  using JointArray = decltype(lbr_fri_msgs::msg::LBRCommand::joint_position);
  using WrenchArray = decltype(lbr_fri_msgs::msg::LBRCommand::wrench);

public:
  LBRIntermediary();

  bool command_to_buffer(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command);
  bool buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const;

  bool state_to_buffer(const KUKA::FRI::LBRState &lbr_state);
  bool buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const;

protected:
  // reset buffers
  bool reset_buffers_();
  bool reset_lbr_command_buffer_();
  bool reset_lbr_state_buffer_();

  // validity checks
  bool valid_lbr_command_(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command) const;
  bool valid_joint_position_command_(const JointArray &joint_position_command) const;
  bool valid_wrench_command_(const WrenchArray &wrench_command) const;
  bool valid_torque_command_(const JointArray &torque_command) const;

  lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command_buffer_;
  lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state_buffer_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
