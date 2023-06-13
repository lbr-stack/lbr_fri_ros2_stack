#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {
LBRClient::LBRClient(const rclcpp::Node::SharedPtr node,
                     const lbr_fri_ros2::LBRCommandGuard &lbr_command_guard)
    : node_(node),
      lbr_command_guard_(std::make_unique<lbr_fri_ros2::LBRCommandGuard>(lbr_command_guard)) {
  lbr_command_rt_buf_ =
      std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>(
          nullptr);
  lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
      "/lbr_command", rclcpp::SensorDataQoS(),
      std::bind(&LBRClient::lbr_command_sub_cb_, this, std::placeholders::_1));
  lbr_state_pub_ =
      node_->create_publisher<lbr_fri_msgs::msg::LBRState>("/lbr_state", rclcpp::SensorDataQoS());
  lbr_state_rt_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(
          lbr_state_pub_);
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
  init_lbr_command_rt_buf_();

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::WRENCH) {
    robotCommand().setWrench(lbr_command_rt_buf_->readFromRT()->get()->wrench.data());
  }

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
    robotCommand().setTorque(lbr_command_rt_buf_->readFromRT()->get()->torque.data());
  }
}

void LBRClient::command() {
  pub_lbr_state_();

  auto lbr_command = lbr_command_rt_buf_->readFromRT();

  // validate command
  if (!lbr_command || !(*lbr_command)) {
    KUKA::FRI::LBRClient::command();
    return;
  }

  if (!lbr_command_guard_->is_valid_command(*lbr_command, robotState())) {
    KUKA::FRI::LBRClient::command();
    return;
  };

  // set command
  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return;
  case KUKA::FRI::EClientCommandMode::POSITION:
    robotCommand().setJointPosition(lbr_command->get()->joint_position.data());
    return;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    robotCommand().setJointPosition(lbr_command->get()->joint_position.data());
    robotCommand().setWrench(lbr_command->get()->wrench.data());
    return;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    robotCommand().setJointPosition(lbr_command->get()->joint_position.data());
    robotCommand().setTorque(lbr_command->get()->torque.data());
    return;
  default:
    printf("Unknown EClientCommandMode provided.\n");
    return;
  }
}

void LBRClient::pub_lbr_state_() {
  if (lbr_state_rt_pub_->trylock()) {
    lbr_state_rt_pub_->msg_.client_command_mode = robotState().getClientCommandMode();
    auto commanded_joint_position = robotState().getCommandedJointPosition();
    std::copy(commanded_joint_position,
              commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_rt_pub_->msg_.commanded_joint_position.begin());
    auto commanded_torque = robotState().getCommandedTorque();
    std::copy(commanded_torque, commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_rt_pub_->msg_.commanded_torque.begin());
    lbr_state_rt_pub_->msg_.connection_quality = robotState().getConnectionQuality();
    lbr_state_rt_pub_->msg_.control_mode = robotState().getControlMode();
    lbr_state_rt_pub_->msg_.drive_state = robotState().getDriveState();
    auto external_torque = robotState().getExternalTorque();
    std::copy(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_rt_pub_->msg_.external_torque.begin());
    if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
        robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
      auto ipo_joint_position = robotState().getIpoJointPosition();
      std::copy(ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                lbr_state_rt_pub_->msg_.ipo_joint_position.begin());
    }
    auto measured_joint_position = robotState().getMeasuredJointPosition();
    std::copy(measured_joint_position,
              measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_rt_pub_->msg_.measured_joint_position.begin());
    auto measured_torque = robotState().getMeasuredTorque();
    std::copy(measured_torque, measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
              lbr_state_rt_pub_->msg_.measured_torque.begin());
    lbr_state_rt_pub_->msg_.operation_mode = robotState().getOperationMode();
    lbr_state_rt_pub_->msg_.overlay_type = robotState().getOverlayType();
    lbr_state_rt_pub_->msg_.safety_state = robotState().getSafetyState();
    lbr_state_rt_pub_->msg_.sample_time = robotState().getSampleTime();
    lbr_state_rt_pub_->msg_.session_state = robotState().getSessionState();
    lbr_state_rt_pub_->msg_.time_stamp_nano_sec = robotState().getTimestampNanoSec();
    lbr_state_rt_pub_->msg_.time_stamp_sec = robotState().getTimestampSec();
    lbr_state_rt_pub_->msg_.tracking_performance = robotState().getTrackingPerformance();

    // publish
    lbr_state_rt_pub_->unlockAndPublish();
  }
}

void LBRClient::lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  lbr_command_rt_buf_->writeFromNonRT(lbr_command);
}

void LBRClient::init_lbr_command_rt_buf_() {
  auto lbr_command = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
  std::copy(robotState().getIpoJointPosition(),
            robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
            lbr_command->joint_position.begin());
  lbr_command->torque.fill(0.);
  lbr_command->wrench.fill(0.);
  lbr_command_rt_buf_->initRT(lbr_command);
}
} // end of namespace lbr_fri_ros2
