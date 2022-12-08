#include "lbr_fri_ros2/lbr_pass_through_client.hpp"

namespace lbr_fri_ros2 {

LBRPassThroughClient::LBRPassThroughClient()
    : LBRPassThroughClient({KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.},
                           {KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.}, {6, 0.}) {}

LBRPassThroughClient::LBRPassThroughClient(const std::vector<double> &delta_joint_position_limit,
                                           const std::vector<double> &torque_limit,
                                           const std::vector<double> &wrench_limit)
    : lbr_command_(nullptr), lbr_state_(nullptr) {
  if (delta_joint_position_limit.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    throw std::length_error("Got delta_joint_position_limit of unexpected size.");
  }
  if (torque_limit.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    throw std::length_error("Got torque_limit of unexpected size.");
  }
  if (wrench_limit.size() != 6) {
    throw std::length_error("Got wrench_limit of unexpected size.");
  }

  this->delta_joint_position_limit = delta_joint_position_limit;
  this->torque_limit = torque_limit;
  this->wrench_limit = wrench_limit;

  init_lbr_fri_msgs_();
}

void LBRPassThroughClient::onStateChange(KUKA::FRI::ESessionState old_state,
                                         KUKA::FRI::ESessionState new_state) {
  printf("LBR switching from %s to %s.\n", e_session_state_to_string(old_state).c_str(),
         e_session_state_to_string(new_state).c_str());
  reset_lbr_fri_msgs_();
}
void LBRPassThroughClient::monitor() {
  KUKA::FRI::LBRClient::monitor();
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
}

void LBRPassThroughClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
}

void LBRPassThroughClient::command() {
  if (!robot_state_to_lbr_state_()) {
    printf("Failed to convert robot state to lbr state.\n");
  }
  if (!lbr_command_to_robot_command_()) {
    printf("Failed to command robot. Defaulting to mirror ipo joint position.\n");
    KUKA::FRI::LBRClient::command();
  }
}

bool LBRPassThroughClient::init_lbr_fri_msgs_() {
  if (!init_lbr_command_()) {
    return false;
  }
  if (!init_lbr_state_()) {
    return false;
  }
  return true;
}

bool LBRPassThroughClient::init_lbr_command_() {
  try {
    lbr_command_ = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
    lbr_command_->joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                        std::numeric_limits<double>::quiet_NaN());
    lbr_command_->torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                std::numeric_limits<double>::quiet_NaN());
    lbr_command_->wrench.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    printf("Failed to initialize lbr_command. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRPassThroughClient::init_lbr_state_() {
  try {
    lbr_state_ = std::make_shared<lbr_fri_msgs::msg::LBRState>();
    lbr_state_->client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->commanded_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                                std::numeric_limits<double>::quiet_NaN());
    lbr_state_->commanded_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                        std::numeric_limits<double>::quiet_NaN());
    lbr_state_->connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->control_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->drive_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->external_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                       std::numeric_limits<double>::quiet_NaN());
    lbr_state_->ipo_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                          std::numeric_limits<double>::quiet_NaN());
    lbr_state_->measured_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                               std::numeric_limits<double>::quiet_NaN());
    lbr_state_->measured_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                                       std::numeric_limits<double>::quiet_NaN());
    lbr_state_->operation_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->safety_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->sample_time = std::numeric_limits<double>::quiet_NaN();
    lbr_state_->session_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_->time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_->tracking_performance = std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception &e) {
    printf("Failed to initialize lbr_state. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRPassThroughClient::reset_lbr_fri_msgs_() {
  if (!reset_lbr_command_()) {
    return false;
  }
  if (!reset_lbr_state_()) {
    return false;
  }
  return true;
}

bool LBRPassThroughClient::reset_lbr_command_() {
  try {
    std::fill(lbr_command_->joint_position.begin(), lbr_command_->joint_position.end(),
              std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_command_->torque.begin(), lbr_command_->torque.end(),
              std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_command_->wrench.begin(), lbr_command_->wrench.end(),
              std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    printf("Failed to reset lbr_command. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRPassThroughClient::reset_lbr_state_() {
  try {
    lbr_state_ = std::make_shared<lbr_fri_msgs::msg::LBRState>();
    lbr_state_->client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
    std::fill(lbr_state_->commanded_joint_position.begin(),
              lbr_state_->commanded_joint_position.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_state_->commanded_torque.begin(), lbr_state_->commanded_torque.end(),
              std::numeric_limits<double>::quiet_NaN());
    lbr_state_->connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->control_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->drive_state = std::numeric_limits<int8_t>::quiet_NaN();
    std::fill(lbr_state_->external_torque.begin(), lbr_state_->external_torque.end(),
              std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_state_->ipo_joint_position.begin(), lbr_state_->ipo_joint_position.end(),
              std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_state_->measured_joint_position.begin(),
              lbr_state_->measured_joint_position.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(lbr_state_->measured_torque.begin(), lbr_state_->measured_torque.end(),
              std::numeric_limits<double>::quiet_NaN());
    lbr_state_->operation_mode = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->safety_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->sample_time = std::numeric_limits<double>::quiet_NaN();
    lbr_state_->session_state = std::numeric_limits<int8_t>::quiet_NaN();
    lbr_state_->time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_->time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
    lbr_state_->tracking_performance = std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception &e) {
    printf("Failed to reset lbr_state. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRPassThroughClient::verify_lbr_command_() {
  if (!lbr_command_) {
    return false;
  }

  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (!valid_joint_position_command_()) {
      printf("Attempted to command invalid joint position.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (!valid_torque_command_()) {
      printf("Attempted to command invalid joint position and or torques.\n");
      return false;
    }
    break;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (!valid_wrench_command_()) {
      printf("Attempted to command invalid joint position and or wrench.\n");
      return false;
    }
    break;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return false;
  }
  return true;
}

bool LBRPassThroughClient::valid_joint_position_command_() {
  if (lbr_command_->joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    printf("Received joint position command of size %lu. Expected %d.\n",
           lbr_command_->joint_position.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  for (std::size_t i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    if (std::isnan(lbr_command_->joint_position[i])) {
      printf("Received nan joint position command on joint %lu.\n", i);
      return false;
    }
    if (std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]) >
        delta_joint_position_limit[i]) {
      printf("Requested position delta command %f on joint %lu. Maximally allowed is %f.\n",
             std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]), i,
             delta_joint_position_limit[i]);
      return false;
    }
  }
  return true;
}

bool LBRPassThroughClient::valid_torque_command_() {
  if (lbr_command_->joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    printf("Received joint position command of size %lu. Expected %d.\n",
           lbr_command_->joint_position.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  if (lbr_command_->torque.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    printf("Received torque command of size %lu. Expected %d.\n", lbr_command_->torque.size(),
           KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  for (std::size_t i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    if (std::isnan(lbr_command_->joint_position[i])) {
      printf("Received nan joint position command on joint %lu.\n", i);
      return false;
    }
    if (std::isnan(lbr_command_->torque[i])) {
      printf("Received nan torque command on joint %lu.\n", i);
      return false;
    }
    if (std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]) >
        delta_joint_position_limit[i]) {
      printf("Requested position delta command %f on joint %lu. Maximally allowed is %f.\n",
             std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]), i,
             delta_joint_position_limit[i]);
      return false;
    }
    if (std::abs(lbr_command_->torque[i]) > torque_limit[i]) {
      printf("Requested torque command %f on joint %lu. Maximally allowed is %f.\n",
             std::abs(lbr_command_->torque[i]), i, torque_limit[i]);
      return false;
    }
  }
  return true;
}

bool LBRPassThroughClient::valid_wrench_command_() {
  if (lbr_command_->joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    printf("Received joint position command of size %lu. Expected %d.\n",
           lbr_command_->joint_position.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  if (lbr_command_->wrench.size() != 6) {
    printf("Received wrench command of size %lu. Expected %d.\n", lbr_command_->wrench.size(), 6);
    return false;
  }
  for (std::size_t i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    if (std::isnan(lbr_command_->joint_position[i])) {
      printf("Received nan joint position command on joint %lu.\n", i);
      return false;
    }
    if (std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]) >
        delta_joint_position_limit[i]) {
      printf("Requested position delta command %f on joint %lu. Maximally allowed is %f.\n",
             std::abs(lbr_command_->joint_position[i] - lbr_state_->measured_joint_position[i]), i,
             delta_joint_position_limit[i]);
      return false;
    }
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::isnan(lbr_command_->wrench[i])) {
      printf("Received nan wrench command on axis %lu.\n", i);
      return false;
    }
    if (std::abs(lbr_command_->wrench[i]) > wrench_limit[i]) {
      printf("Requested wrench command %f on axis %lu. Maximally allowed is %f.\n",
             std::abs(lbr_command_->wrench[i]), i, wrench_limit[i]);
      return false;
    }
  }
  return true;
}

bool LBRPassThroughClient::robot_state_to_lbr_state_() {
  try {
    lbr_state_->client_command_mode = robotState().getClientCommandMode();
    auto commanded_joint_position = robotState().getCommandedJointPosition();
    lbr_state_->commanded_joint_position.assign(
        commanded_joint_position, commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    auto commanded_torque = robotState().getCommandedTorque();
    lbr_state_->commanded_torque.assign(commanded_torque,
                                        commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    lbr_state_->connection_quality = robotState().getConnectionQuality();
    lbr_state_->control_mode = robotState().getControlMode();
    lbr_state_->drive_state = robotState().getDriveState();
    auto external_torque = robotState().getExternalTorque();
    lbr_state_->external_torque.assign(external_torque,
                                       external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    auto ipo_joint_position = robotState().getIpoJointPosition();
    lbr_state_->ipo_joint_position.assign(
        ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    auto measured_joint_position = robotState().getMeasuredJointPosition();
    lbr_state_->measured_joint_position.assign(
        measured_joint_position, measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    auto measured_torque = robotState().getMeasuredTorque();
    lbr_state_->measured_torque.assign(measured_torque,
                                       measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    lbr_state_->operation_mode = robotState().getOperationMode();
    lbr_state_->overlay_type = robotState().getOverlayType();
    lbr_state_->safety_state = robotState().getSafetyState();
    lbr_state_->sample_time = robotState().getSampleTime();
    lbr_state_->session_state = robotState().getSessionState();
    lbr_state_->time_stamp_nano_sec = robotState().getTimestampNanoSec();
    lbr_state_->time_stamp_sec = robotState().getTimestampSec();
    lbr_state_->tracking_performance = robotState().getTrackingPerformance();
  } catch (const std::exception &e) {
    printf("Failed to convert robot state to lbr state. %s.\n", e.what());
    return false;
  }
  return true;
}

bool LBRPassThroughClient::lbr_command_to_robot_command_() {
  try {
    if (verify_lbr_command_()) {
      switch (robotState().getClientCommandMode()) {
      case KUKA::FRI::EClientCommandMode::POSITION:
        robotCommand().setJointPosition(lbr_command_->joint_position.data());
        break;
      case KUKA::FRI::EClientCommandMode::TORQUE:
        robotCommand().setJointPosition(lbr_command_->joint_position.data());
        robotCommand().setTorque(lbr_command_->torque.data());
        break;
      case KUKA::FRI::EClientCommandMode::WRENCH:
        robotCommand().setJointPosition(lbr_command_->joint_position.data());
        robotCommand().setWrench(lbr_command_->wrench.data());
        break;
      default:
        printf("Unknown EClientCommandMode provided.\n");
        return false;
      }
    } else {
      reset_lbr_command_();
      return false;
    }
  } catch (const std::exception &e) {
    printf("Failed to convert robot command to lbr command. %s.\n", e.what());
    return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
