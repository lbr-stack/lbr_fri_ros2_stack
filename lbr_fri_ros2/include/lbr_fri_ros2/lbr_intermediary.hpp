#ifndef LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
#define LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_

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
  LBRIntermediary() = default;

  bool zero_command_buffer(const KUKA::FRI::LBRState &lbr_state);
  bool command_to_buffer(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command);
  bool buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const;

  bool state_to_buffer(const KUKA::FRI::LBRState &lbr_state);
  bool buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const;

protected:
  lbr_fri_msgs::msg::LBRCommand lbr_command_buffer_;
  lbr_fri_msgs::msg::LBRState lbr_state_buffer_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
