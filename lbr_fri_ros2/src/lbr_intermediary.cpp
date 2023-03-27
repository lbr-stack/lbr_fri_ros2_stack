#include "lbr_fri_ros2/lbr_intermediary.hpp"

namespace lbr_fri_ros2 {
bool LBRIntermediary::command_to_buffer(
    const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command) {
  if (!lbr_command) {
    return false;
  }
  lbr_command_buffer_ = *lbr_command;
  return true;
}

bool LBRIntermediary::buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const {
  try {
    if (lbr_state_buffer_.client_command_mode == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
      switch (lbr_state_buffer_.client_command_mode) {
      case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
        return true;
      case KUKA::FRI::EClientCommandMode::POSITION:
        lbr_command.setJointPosition(lbr_command_buffer_.joint_position.data());
        return true;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        lbr_command.setJointPosition(lbr_command_buffer_.joint_position.data());
        lbr_command.setWrench(lbr_command_buffer_.wrench.data());
        return true;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        lbr_command.setJointPosition(lbr_command_buffer_.joint_position.data());
        lbr_command.setTorque(lbr_command_buffer_.torque.data());
        return true;
      default:
        printf("Unknown EClientCommandMode provided.\n");
        return false;
      }
    } else {
      switch (lbr_state_buffer_.client_command_mode) {
      case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
        return true;
      case KUKA::FRI::EClientCommandMode::POSITION:
        lbr_command.setJointPosition(lbr_state_buffer_.commanded_joint_position.data());
        return true;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        lbr_command.setJointPosition(lbr_state_buffer_.commanded_joint_position.data());
        lbr_command.setWrench(WRENCH_ZEROS.data());
        return true;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        lbr_command.setJointPosition(lbr_state_buffer_.commanded_joint_position.data());
        lbr_command.setTorque(JOINT_ZEROS.data());
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
    lbr_state_buffer_.client_command_mode = lbr_state.getClientCommandMode();
    auto commanded_joint_position = lbr_state.getCommandedJointPosition();
    std::copy(commanded_joint_position,
              commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_.commanded_joint_position.begin());
    auto commanded_torque = lbr_state.getCommandedTorque();
    std::copy(commanded_torque, commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_.commanded_torque.begin());
    lbr_state_buffer_.connection_quality = lbr_state.getConnectionQuality();
    lbr_state_buffer_.control_mode = lbr_state.getControlMode();
    lbr_state_buffer_.drive_state = lbr_state.getDriveState();
    auto external_torque = lbr_state.getExternalTorque();
    std::copy(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_.external_torque.begin());
    if (lbr_state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
        lbr_state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
      auto ipo_joint_position = lbr_state.getIpoJointPosition();
      std::copy(ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                lbr_state_buffer_.ipo_joint_position.begin());
    }
    auto measured_joint_position = lbr_state.getMeasuredJointPosition();
    std::copy(measured_joint_position,
              measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_.measured_joint_position.begin());
    auto measured_torque = lbr_state.getMeasuredTorque();
    std::copy(measured_torque, measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_buffer_.measured_torque.begin());
    lbr_state_buffer_.operation_mode = lbr_state.getOperationMode();
    lbr_state_buffer_.overlay_type = lbr_state.getOverlayType();
    lbr_state_buffer_.safety_state = lbr_state.getSafetyState();
    lbr_state_buffer_.sample_time = lbr_state.getSampleTime();
    lbr_state_buffer_.session_state = lbr_state.getSessionState();
    lbr_state_buffer_.time_stamp_nano_sec = lbr_state.getTimestampNanoSec();
    lbr_state_buffer_.time_stamp_sec = lbr_state.getTimestampSec();
    lbr_state_buffer_.tracking_performance = lbr_state.getTrackingPerformance();
  } catch (const std::exception &e) {
    printf("Failed to move state to buffer.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRIntermediary::buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const {
  try {
    lbr_state = lbr_state_buffer_;
  } catch (const std::exception &e) {
    printf("Failed to move buffer to state.\n%s", e.what());
    return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
