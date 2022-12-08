#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <realtime_tools/realtime_buffer.h>

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

#include "fri/friLBRClient.h"
#include "fri/friLBRState.h"

#include "lbr_fri_ros2/utilities.hpp"

namespace lbr_fri_ros2 {
class LBRPassThroughClient : public KUKA::FRI::LBRClient {
public:
  LBRPassThroughClient();
  LBRPassThroughClient(const std::vector<double> &delta_joint_position_limit,
                       const std::vector<double> &torque_limit,
                       const std::vector<double> &wrench_limit);

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

  // command setter and state getter for control by owning object
  inline lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command() { return lbr_command_; };
  inline const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state() const { return lbr_state_; };

  // limits for commands
  std::vector<double> delta_joint_position_limit;
  std::vector<double> torque_limit;
  std::vector<double> wrench_limit;

protected:
  // initialize
  bool init_lbr_fri_msgs_();
  bool init_lbr_command_();
  bool init_lbr_state_();

  // reset
  bool reset_lbr_fri_msgs_();
  bool reset_lbr_command_();
  bool reset_lbr_state_();

  // verify command
  bool verify_lbr_command_();
  bool valid_joint_position_command_();
  bool valid_torque_command_();
  bool valid_wrench_command_();

  // pass through from lbr_fri_msgs to robot
  bool robot_state_to_lbr_state_();
  bool lbr_command_to_robot_command_();

  lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command_;
  lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state_;
};
} // end of namespace lbr_fri_ros2
