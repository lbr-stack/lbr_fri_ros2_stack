#pragma once

#include <limits>
#include <stdexcept>

#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
struct LBR {
  static constexpr uint8_t JOINT_DOF = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
  static constexpr uint8_t CARTESIAN_DOF = 6;

  LBR();

  // initialize
  bool init_lbr_fri_msgs();
  bool init_lbr_command();
  bool init_lbr_state();

  // command and state
  lbr_fri_msgs::msg::LBRCommand::SharedPtr command;
  lbr_fri_msgs::msg::LBRState::SharedPtr state;
};
} // end of namespace lbr_fri_ros2
