#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {

LBRClient::LBRClient(std::shared_ptr<LBR> lbr) : lbr_(lbr){};

void LBRClient::onStateChange(KUKA::FRI::ESessionState old_state,
                              KUKA::FRI::ESessionState new_state) {
  printf("LBR switched from %s to %s.\n", session_state_to_string(old_state).c_str(),
         session_state_to_string(new_state).c_str());
  reset_lbr_command_();
}
void LBRClient::monitor() {
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
}

void LBRClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
  if (!lbr_command_to_robot_command_()) {
    printf("Failed to convert lbr command to robot command.\n");
  }
}

void LBRClient::command() {
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
  if (!lbr_command_to_robot_command_()) {
    printf("Failed to convert lbr command to robot command.\n");
  }
}

bool LBRClient::reset_lbr_command_() {
  try {
    lbr_->command->joint_position = lbr_->state->measured_joint_position;
    std::fill(lbr_->command->wrench.begin(), lbr_->command->wrench.end(), 0.);
    std::fill(lbr_->command->torque.begin(), lbr_->command->torque.end(), 0.);
  } catch (const std::exception &e) {
    printf("Failed to reset lbr_command. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRClient::robot_state_to_lbr_state_() {
  try {
    lbr_->state->client_command_mode = robotState().getClientCommandMode();
    auto commanded_joint_position = robotState().getCommandedJointPosition();
    lbr_->state->commanded_joint_position.assign(commanded_joint_position,
                                                 commanded_joint_position + LBR::JOINT_DOF);
    auto commanded_torque = robotState().getCommandedTorque();
    lbr_->state->commanded_torque.assign(commanded_torque, commanded_torque + LBR::JOINT_DOF);
    lbr_->state->connection_quality = robotState().getConnectionQuality();
    lbr_->state->control_mode = robotState().getControlMode();
    lbr_->state->drive_state = robotState().getDriveState();
    auto external_torque = robotState().getExternalTorque();
    lbr_->state->external_torque.assign(external_torque, external_torque + LBR::JOINT_DOF);
    if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
        robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
      auto ipo_joint_position = robotState().getIpoJointPosition();
      lbr_->state->ipo_joint_position.assign(ipo_joint_position,
                                             ipo_joint_position + LBR::JOINT_DOF);
    }
    auto measured_joint_position = robotState().getMeasuredJointPosition();
    lbr_->state->measured_joint_position.assign(measured_joint_position,
                                                measured_joint_position + LBR::JOINT_DOF);
    auto measured_torque = robotState().getMeasuredTorque();
    lbr_->state->measured_torque.assign(measured_torque, measured_torque + LBR::JOINT_DOF);
    lbr_->state->operation_mode = robotState().getOperationMode();
    lbr_->state->overlay_type = robotState().getOverlayType();
    lbr_->state->safety_state = robotState().getSafetyState();
    lbr_->state->sample_time = robotState().getSampleTime();
    lbr_->state->session_state = robotState().getSessionState();
    lbr_->state->time_stamp_nano_sec = robotState().getTimestampNanoSec();
    lbr_->state->time_stamp_sec = robotState().getTimestampSec();
    lbr_->state->tracking_performance = robotState().getTrackingPerformance();
  } catch (const std::exception &e) {
    printf("Failed to convert robot state to lbr state. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRClient::lbr_command_to_robot_command_() {
  try {
    if (lbr_->valid_command(lbr_->command)) {
      switch (robotState().getClientCommandMode()) {
      case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
        printf("Attempted to command when EClientCommandMode::NO_COMMAND_MODE provided.\n");
        return false;
      case KUKA::FRI::EClientCommandMode::POSITION:
        robotCommand().setJointPosition(lbr_->command->joint_position.data());
        break;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        robotCommand().setJointPosition(lbr_->command->joint_position.data());
        robotCommand().setWrench(lbr_->command->wrench.data());
        break;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        robotCommand().setJointPosition(lbr_->command->joint_position.data());
        robotCommand().setTorque(lbr_->command->torque.data());
        break;
      default:
        printf("Unknown EClientCommandMode provided.\n");
        return false;
      }
    } else {
      return false;
    }
  } catch (const std::exception &e) {
    printf("Failed to convert robot command to lbr command. %s.\n", e.what());
    return false;
  }
  return true;
}

std::string LBRClient::session_state_to_string(const KUKA::FRI::ESessionState &state) {
  switch (state) {
  case KUKA::FRI::ESessionState::IDLE:
    return "IDLE";
  case KUKA::FRI::ESessionState::MONITORING_WAIT:
    return "MONITORING_WAIT";
  case KUKA::FRI::ESessionState::MONITORING_READY:
    return "MONITORING_READY";
  case KUKA::FRI::ESessionState::COMMANDING_WAIT:
    return "COMMANDING_WAIT";
  case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
    return "COMMANDING_ACTIVE";
  default:
    throw std::runtime_error("Reveived unknown state.");
  }
}
} // end of namespace lbr_fri_ros2
