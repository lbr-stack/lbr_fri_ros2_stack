#include "lbr_fri_ros2/lbr.hpp"

namespace lbr_fri_ros2 {
LBR::LBR() {
  if (!init_lbr_fri_msgs()) {
    throw std::runtime_error("Failed to initialize lbr_fri_msgs.");
  }
}

bool LBR::init_lbr_fri_msgs() {
  if (!init_lbr_command()) {
    return false;
  }
  if (!init_lbr_state()) {
    return false;
  }
  return true;
}

bool LBR::init_lbr_command() {
  try {
    command = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
    command->joint_position.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    command->torque.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    command->wrench.resize(CARTESIAN_DOF, std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    printf("Failed to initialize command. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBR::init_lbr_state() {
  try {
    state = std::make_shared<lbr_fri_msgs::msg::LBRState>();
    state->client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
    state->commanded_joint_position.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->commanded_torque.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
    state->control_mode = std::numeric_limits<int8_t>::quiet_NaN();
    state->drive_state = std::numeric_limits<int8_t>::quiet_NaN();
    state->external_torque.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->ipo_joint_position.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->measured_joint_position.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->measured_torque.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    state->operation_mode = std::numeric_limits<int8_t>::quiet_NaN();
    state->overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
    state->safety_state = std::numeric_limits<int8_t>::quiet_NaN();
    state->sample_time = std::numeric_limits<double>::quiet_NaN();
    state->session_state = std::numeric_limits<int8_t>::quiet_NaN();
    state->time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    state->time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    state->tracking_performance = std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception &e) {
    printf("Failed to initialize state. %s.\n", e.what());
    return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
