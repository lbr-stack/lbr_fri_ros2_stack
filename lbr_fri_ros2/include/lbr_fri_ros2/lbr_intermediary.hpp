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
  using JointArray = decltype(lbr_fri_msgs::msg::LBRCommand::joint_position);
  using WrenchArray = decltype(lbr_fri_msgs::msg::LBRCommand::wrench);

  static constexpr auto JOINT_ZEROS = JointArray{0.};
  static constexpr auto WRENCH_ZEROS = WrenchArray{0.};

public:
  LBRIntermediary();

  bool command_to_buffer(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command);
  bool buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const;

  bool state_to_buffer(const KUKA::FRI::LBRState &lbr_state);
  bool buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const;

  void set_lbr_command_buffer_nan();
  void set_lbr_state_buffer_nan();

protected:
  // reset buffers
  bool reset_buffers_();
  bool reset_lbr_command_buffer_();
  bool reset_lbr_state_buffer_();

  // validity checks
  bool lbr_command_is_nan_(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command) const;
  bool joint_position_command_is_nan_(const JointArray &joint_position_command) const;
  bool wrench_command_is_nan_(const WrenchArray &wrench_command) const;
  bool torque_command_is_nan_(const JointArray &torque_command) const;

  bool lbr_state_is_nan_(const lbr_fri_msgs::msg::LBRState::ConstSharedPtr lbr_state) const;

  lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command_buffer_;
  lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state_buffer_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
