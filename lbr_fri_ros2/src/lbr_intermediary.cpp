#include "lbr_fri_ros2/lbr_intermediary.hpp"

namespace lbr_fri_ros2 {

LBRIntermediary::LBRIntermediary() {
  if (!reset_buffers_()) {
    throw std::runtime_error("Failed to reset buffers.");
  }
}

bool LBRIntermediary::reset_buffers_() {
  if (!reset_lbr_command_buffer_()) {
    return false;
  }
  if (!reset_lbr_state_buffer_()) {
    return false;
  }
  return true;
}

bool LBRIntermediary::reset_lbr_command_buffer_() {
  try {
    lbr_command_buffer_ = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
    lbr_command_buffer_->joint_position.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_command_buffer_->wrench.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_command_buffer_->torque.fill(std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    printf("Failed to reset command buffer.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRIntermediary::reset_lbr_state_buffer_() {
  try {
    lbr_state_buffer_ = std::make_shared<lbr_fri_msgs::msg::LBRState>();
    lbr_state_buffer_->client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->commanded_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->commanded_torque.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->control_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->drive_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->external_torque.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->ipo_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->measured_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->measured_torque.fill(std::numeric_limits<double>::quiet_NaN());
    lbr_state_buffer_->operation_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->safety_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->sample_time = std::numeric_limits<double>::quiet_NaN();
    lbr_state_buffer_->session_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_buffer_->time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_buffer_->time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_buffer_->tracking_performance = std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception &e) {
    printf("Failed to reset state buffer.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRIntermediary::command_to_buffer(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command) {
  if (valid_lbr_command_(lbr_command)) {
    *lbr_command_buffer_ = *lbr_command;
    return true;
  }
  return false;
}

bool LBRIntermediary::buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const {
  try {
    if (valid_lbr_command_(lbr_command_buffer_)) {
      switch (lbr_state_buffer_->client_command_mode) {
      case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
        return true;
      case KUKA::FRI::EClientCommandMode::POSITION:
        lbr_command.setJointPosition(lbr_command_buffer_->joint_position.data());
        return true;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        lbr_command.setJointPosition(lbr_command_buffer_->joint_position.data());
        lbr_command.setWrench(lbr_command_buffer_->wrench.data());
        return true;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        lbr_command.setJointPosition(lbr_command_buffer_->joint_position.data());
        lbr_command.setTorque(lbr_command_buffer_->torque.data());
        return true;
      default:
        printf("Unknown EClientCommandMode provided.\n");
        return false;
      }
    } else {
      switch (lbr_state_buffer_->client_command_mode) {
      case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
        return true;
      case KUKA::FRI::EClientCommandMode::POSITION:
        lbr_command.setJointPosition(lbr_state_buffer_->measured_joint_position.data());
        return true;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        lbr_command.setJointPosition(lbr_state_buffer_->measured_joint_position.data());
        // lbr_command.setJointPosition(lbr_state_buffer_->measured_joint_position());
        return true;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        lbr_command.setJointPosition(lbr_state_buffer_->measured_joint_position.data());
        // lbr_command.setJointPosition(lbr_state_buffer_->measured_joint_position());
        return true;
      default:
        printf("Unknown EClientCommandMode provided.\n");
        return false;
      }
    }
  } catch (const std::exception &e) {
    printf("Failed to move buffer to command.\n%s", e.what());
  }
  return false;
}

bool LBRIntermediary::state_to_buffer(const KUKA::FRI::LBRState &lbr_state) {
  try {
    lbr_state_buffer_->client_command_mode = lbr_state.getClientCommandMode();
    auto commanded_joint_position = lbr_state.getCommandedJointPosition();
    std::copy(commanded_joint_position,
              commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_->commanded_joint_position.begin());
    auto commanded_torque = lbr_state.getCommandedTorque();
    std::copy(commanded_torque, commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_->commanded_torque.begin());
    lbr_state_buffer_->connection_quality = lbr_state.getConnectionQuality();
    lbr_state_buffer_->control_mode = lbr_state.getControlMode();
    lbr_state_buffer_->drive_state = lbr_state.getDriveState();
    auto external_torque = lbr_state.getExternalTorque();
    std::copy(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_->external_torque.begin());
    if (lbr_state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
        lbr_state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
      auto ipo_joint_position = lbr_state.getIpoJointPosition();
      std::copy(ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                lbr_state_buffer_->ipo_joint_position.begin());
    }
    auto measured_joint_position = lbr_state.getMeasuredJointPosition();
    std::copy(measured_joint_position,
              measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_->measured_joint_position.begin());
    auto measured_torque = lbr_state.getMeasuredTorque();
    std::copy(measured_torque, measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_->measured_torque.begin());
    lbr_state_buffer_->operation_mode = lbr_state.getOperationMode();
    lbr_state_buffer_->overlay_type = lbr_state.getOverlayType();
    lbr_state_buffer_->safety_state = lbr_state.getSafetyState();
    lbr_state_buffer_->sample_time = lbr_state.getSampleTime();
    lbr_state_buffer_->session_state = lbr_state.getSessionState();
    lbr_state_buffer_->time_stamp_nano_sec = lbr_state.getTimestampNanoSec();
    lbr_state_buffer_->time_stamp_sec = lbr_state.getTimestampSec();
    lbr_state_buffer_->tracking_performance = lbr_state.getTrackingPerformance();
  } catch (const std::exception &e) {
    printf("Failed to move state to buffer.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRIntermediary::buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const {
  try {
    lbr_state = *lbr_state_buffer_;
  } catch (const std::exception &e) {
    printf("Failed to move buffer to state.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRIntermediary::valid_lbr_command_(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command) const {
  if (!lbr_command) {
    return false;
  }

  switch (lbr_state_buffer_->client_command_mode) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return true;
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (!valid_joint_position_command_(lbr_command->joint_position)) {
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (!valid_joint_position_command_(lbr_command->joint_position)) {
      return false;
    }
    if (!valid_wrench_command_(lbr_command->wrench)) {
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (!valid_joint_position_command_(lbr_command->joint_position)) {
      return false;
    }
    if (!valid_torque_command_(lbr_command->torque)) {
      return false;
    }
    break;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return false;
  }
  return true;
}

bool LBRIntermediary::valid_joint_position_command_(
    const JointArray &joint_position_command) const {
  for (std::size_t i = 0; i < joint_position_command.size(); ++i) {
    if (std::isnan(joint_position_command[i])) {
      return false;
    }
  }
  return true;
}

bool LBRIntermediary::valid_wrench_command_(const WrenchArray &wrench_command) const {
  for (std::size_t i = 0; i < wrench_command.size(); ++i) {
    if (std::isnan(wrench_command[i])) {
      return false;
    }
  }
  return true;
}

bool LBRIntermediary::valid_torque_command_(const JointArray &torque_command) const {
  for (std::size_t i = 0; i < torque_command.size(); ++i) {
    if (std::isnan(torque_command[i])) {
      return false;
    }
  }
  return true;
}
} // end of namespace lbr_fri_ros2
