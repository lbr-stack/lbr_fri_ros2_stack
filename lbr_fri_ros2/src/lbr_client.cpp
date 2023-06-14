#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {
LBRClient::LBRClient(const rclcpp::Node::SharedPtr node,
                     std::unique_ptr<LBRCommandGuard> lbr_command_guard)
    : node_(node), lbr_command_guard_(std::move(lbr_command_guard)) {
  lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
      "/lbr_command", rclcpp::SensorDataQoS(),
      std::bind(&LBRClient::lbr_command_sub_cb_, this, std::placeholders::_1));
  lbr_state_pub_ =
      node_->create_publisher<lbr_fri_msgs::msg::LBRState>("/lbr_state", rclcpp::SensorDataQoS());
}

void LBRClient::onStateChange(KUKA::FRI::ESessionState old_state,
                              KUKA::FRI::ESessionState new_state) {
  printf("LBR switched from %s to %s.\n", KUKA_FRI_STATE_MAP[old_state].c_str(),
         KUKA_FRI_STATE_MAP[new_state].c_str());
}
void LBRClient::monitor() { pub_lbr_state_(); }

void LBRClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  pub_lbr_state_();
  init_lbr_command_();

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::WRENCH) {
    robotCommand().setWrench(lbr_command_.wrench.data());
  }

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
    robotCommand().setTorque(lbr_command_.torque.data());
  }
}

void LBRClient::command() {
  pub_lbr_state_();

  // validate command
  if (!lbr_command_guard_->is_valid_command(lbr_command_, robotState())) {
    // not responding -> connection lost
    return;
  };

  // set command
  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return;
  case KUKA::FRI::EClientCommandMode::POSITION:
    robotCommand().setJointPosition(lbr_command_.joint_position.data());
    return;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    robotCommand().setJointPosition(lbr_command_.joint_position.data());
    robotCommand().setWrench(lbr_command_.wrench.data());
    return;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    robotCommand().setJointPosition(lbr_command_.joint_position.data());
    robotCommand().setTorque(lbr_command_.torque.data());
    return;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return;
  }
}

void LBRClient::pub_lbr_state_() {
  lbr_state_.client_command_mode = robotState().getClientCommandMode();
  auto commanded_joint_position = robotState().getCommandedJointPosition();
  std::copy(commanded_joint_position,
            commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_state_.commanded_joint_position.begin());
  auto commanded_torque = robotState().getCommandedTorque();
  std::copy(commanded_torque, commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_state_.commanded_torque.begin());
  lbr_state_.connection_quality = robotState().getConnectionQuality();
  lbr_state_.control_mode = robotState().getControlMode();
  lbr_state_.drive_state = robotState().getDriveState();
  auto external_torque = robotState().getExternalTorque();
  std::copy(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_state_.external_torque.begin());
  if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    auto ipo_joint_position = robotState().getIpoJointPosition();
    std::copy(ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_.ipo_joint_position.begin());
  }
  auto measured_joint_position = robotState().getMeasuredJointPosition();
  std::copy(measured_joint_position,
            measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_state_.measured_joint_position.begin());
  auto measured_torque = robotState().getMeasuredTorque();
  std::copy(measured_torque, measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_state_.measured_torque.begin());
  lbr_state_.operation_mode = robotState().getOperationMode();
  lbr_state_.overlay_type = robotState().getOverlayType();
  lbr_state_.safety_state = robotState().getSafetyState();
  lbr_state_.sample_time = robotState().getSampleTime();
  lbr_state_.session_state = robotState().getSessionState();
  lbr_state_.time_stamp_nano_sec = robotState().getTimestampNanoSec();
  lbr_state_.time_stamp_sec = robotState().getTimestampSec();
  lbr_state_.tracking_performance = robotState().getTrackingPerformance();

  // publish
  lbr_state_pub_->publish(lbr_state_);
}

void LBRClient::lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    lbr_command_.joint_position[i] = filters::exponentialSmoothing(
        lbr_command_.joint_position[i], lbr_command->joint_position[i], 0.98);
  }
  lbr_command_.torque = lbr_command->torque;
  lbr_command_.wrench = lbr_command->wrench;
}

void LBRClient::init_lbr_command_() {
  std::copy(robotState().getIpoJointPosition(),
            robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_command_.joint_position.begin());
  lbr_command_.torque.fill(0.);
  lbr_command_.wrench.fill(0.);
}
} // end of namespace lbr_fri_ros2
