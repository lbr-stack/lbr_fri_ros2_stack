#include "lbr_fri_ros2/lbr.hpp"

namespace lbr_fri_ros2 {
LBR::LBR() {
  if (!init_fri_msgs()) {
    throw std::runtime_error("Failed to initialize lbr_fri_msgs.");
  }
}

bool LBR::init_fri_msgs() {
  if (!init_command()) {
    return false;
  }
  if (!init_state()) {
    return false;
  }
  if (!init_limits()) {
    return false;
  }
  return true;
}

bool LBR::init_command() {
  try {
    command = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
    command->joint_position.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    command->wrench.resize(CARTESIAN_DOF, std::numeric_limits<double>::quiet_NaN());
    command->torque.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    printf("Failed to initialize command. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBR::init_state() {
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

bool LBR::init_limits() {
  try {
    joint_velocity_command_limit.resize(JOINT_DOF, 0.);
    wrench_command_limit.resize(CARTESIAN_DOF, 0.);
    torque_command_limit.resize(JOINT_DOF, 0.);
  } catch (const std::exception &e) {
    printf("Failed to initialize limits.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBR::valid_command() {
  if (!command) {
    return false;
  }

  switch (state->client_command_mode) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return true;
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (!valid_joint_position_command()) {
      printf("Attempted to command invalid joint position.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (!valid_wrench_command()) {
      printf("Attempted to command invalid joint position and or wrench.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (!valid_torque_command()) {
      printf("Attempted to command invalid joint position and or torques.\n");
      return false;
    }
    break;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return false;
  }
  return true;
}

bool LBR::valid_joint_position_command() {
  if (command->joint_position.size() != JOINT_DOF) {
    printf("Received joint position command of size %lu. Expected %d.\n",
           command->joint_position.size(), JOINT_DOF);
    return false;
  }
  for (std::size_t i = 0; i < JOINT_DOF; ++i) {
    if (std::isnan(command->joint_position[i])) {
      printf("Received nan joint position command on joint %lu.\n", i);
      return false;
    }
  }
  return true;
}

bool LBR::valid_wrench_command() {
  if (command->wrench.size() != CARTESIAN_DOF) {
    printf("Received wrench command of size %lu. Expected %d.\n", command->wrench.size(),
           CARTESIAN_DOF);
    return false;
  }
  for (std::size_t i = 0; i < CARTESIAN_DOF; ++i) {
    if (std::isnan(command->wrench[i])) {
      printf("Received nan wrench command on axis %lu.\n", i);
      return false;
    }
  }
  return true;
}

bool LBR::valid_torque_command() {
  if (command->torque.size() != JOINT_DOF) {
    printf("Received torque command of size %lu. Expected %d.\n", command->torque.size(),
           JOINT_DOF);
    return false;
  }
  for (std::size_t i = 0; i < JOINT_DOF; ++i) {
    if (std::isnan(command->torque[i])) {
      printf("Received nan torque command on joint %lu.\n", i);
      return false;
    }
  }
  return true;
}

bool LBR::valid_state() {
  if (!state) {
    printf("Found no state.\n");
    return false;
  }
  if (state->commanded_joint_position.size() != JOINT_DOF ||
      state->commanded_torque.size() != JOINT_DOF || state->external_torque.size() != JOINT_DOF ||
      state->ipo_joint_position.size() != JOINT_DOF ||
      state->measured_joint_position.size() != JOINT_DOF ||
      state->measured_torque.size() != JOINT_DOF) {
    printf("State of invalid size found.\n");
    return false;
  }
  for (uint8_t i = 0; i < JOINT_DOF; ++i) {
    if (std::isnan(state->commanded_joint_position[i]) || std::isnan(state->commanded_torque[i]) ||
        std::isnan(state->external_torque[i]) || std::isnan(state->measured_joint_position[i]) ||
        std::isnan(state->measured_torque[i])) {
      printf("Found nan for state in joint %d.\n", i);
      return false;
    }
  }
  if (state->session_state == KUKA::FRI::COMMANDING_WAIT ||
      state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
    for (uint8_t i = 0; i < JOINT_DOF; ++i) {
      if (std::isnan(state->ipo_joint_position[i])) {
        printf("Found nan for interpolated joint position in joint %d.\n", i);
        return false;
      }
    }
  }
  if (std::isnan(state->client_command_mode) || std::isnan(state->connection_quality) ||
      std::isnan(state->control_mode) || std::isnan(state->drive_state) ||
      std::isnan(state->operation_mode) || std::isnan(state->overlay_type) ||
      std::isnan(state->safety_state) || std::isnan(state->sample_time) ||
      std::isnan(state->session_state) || std::isnan(state->time_stamp_nano_sec) ||
      std::isnan(state->time_stamp_sec) || std::isnan(state->tracking_performance)) {
    printf("Found nan in other state variables.\n");
    return false;
  }
  return true;
}

bool LBR::command_within_limits(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  if (!lbr_command) {
    return false;
  }

  switch (state->client_command_mode) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return true;
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (!joint_position_command_within_limits(lbr_command->joint_position)) {
      printf("Desired joint position command outside of velocity limits.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (!wrench_command_within_limits(lbr_command->wrench)) {
      printf("Desired joint position command and or wrench command outside limits.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (!torque_command_within_limits(lbr_command->torque)) {
      printf("Desired joint position command and or torque command outside limits.\n");
      return false;
    }
    break;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return false;
  }
  return true;
}

bool LBR::joint_position_command_within_limits(const std::vector<double> &joint_position_command) {
  if (joint_position_command.size() != LBR::JOINT_DOF) {
    printf("Joint position command of size %lu received. Expected size %d.\n",
           joint_position_command.size(), LBR::JOINT_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::JOINT_DOF; ++i) {
    if (std::abs(joint_position_command[i] - state->measured_joint_position[i]) >
        joint_velocity_command_limit[i] * state->sample_time) {
      printf("Got joint velocity command abs(%f) on joint %d. Limit is %f. Current position is %f, "
             "desired position is %f.\n",
             std::abs(joint_position_command[i] - state->measured_joint_position[i]) /
                 state->sample_time,
             i, joint_velocity_command_limit[i], state->measured_joint_position[i],
             joint_position_command[i]);
      return false;
    }
  }
  return true;
}

bool LBR::wrench_command_within_limits(const std::vector<double> &wrench_command) {
  if (wrench_command.size() != LBR::CARTESIAN_DOF) {
    printf("Wrench command of size %lu received. Expected size %d.\n", wrench_command.size(),
           LBR::CARTESIAN_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::CARTESIAN_DOF; ++i) {
    if (std::abs(wrench_command[i]) > wrench_command_limit[i]) {
      printf("Got wrench command abs(%f) on axis %d. Limit is %f.\n", std::abs(wrench_command[i]),
             i, wrench_command_limit[i]);
      return false;
    }
  }
  return true;
}

bool LBR::torque_command_within_limits(const std::vector<double> &torque_command) {
  if (torque_command.size() != LBR::JOINT_DOF) {
    printf("Torque command of size %lu received. Expected size %d.\n", torque_command.size(),
           LBR::JOINT_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::JOINT_DOF; ++i) {
    if (std::abs(torque_command[i]) > torque_command_limit[i]) {
      printf("Got torque command abs(%f) on joint %d. Limit is %f.\n", std::abs(torque_command[i]),
             i, torque_command_limit[i]);
      return false;
    }
  }
  return true;
}
} // end of namespace lbr_fri_ros2
