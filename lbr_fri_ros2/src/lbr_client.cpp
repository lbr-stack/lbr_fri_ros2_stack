#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {
LBRClient::LBRClient(const rclcpp::Node::SharedPtr node,
                     std::unique_ptr<LBRCommandGuard> lbr_command_guard)
    : node_(node), lbr_command_guard_(std::move(lbr_command_guard)) {

  missed_deadlines_pub_ = 0;
  missed_deadlines_sub_ = 0;

  declare_parameters_();
  get_parameters_();
}

void LBRClient::log_status() {
  RCLCPP_INFO(node_->get_logger(), "LBRClient - Publisher missed deadlines: %u",
              missed_deadlines_pub_);
  RCLCPP_INFO(node_->get_logger(), "LBRClient - Subscription missed deadlines: %u",
              missed_deadlines_sub_);
}

void LBRClient::onStateChange(KUKA::FRI::ESessionState old_state,
                              KUKA::FRI::ESessionState new_state) {
  init_topics_();
  RCLCPP_INFO(node_->get_logger(), "LBR switched from %s to %s.",
              KUKA_FRI_STATE_MAP[old_state].c_str(), KUKA_FRI_STATE_MAP[new_state].c_str());
}
void LBRClient::monitor() {
  pub_lbr_state_();
  init_lbr_command_();
}

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
    RCLCPP_ERROR(node_->get_logger(), "Invalid command received. Triggering disconnect.");
    return;
  };

  // set command
  robotCommand().setJointPosition(lbr_command_.joint_position.data());
  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
  case KUKA::FRI::EClientCommandMode::POSITION:
    return;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    robotCommand().setWrench(lbr_command_.wrench.data());
    return;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    robotCommand().setTorque(lbr_command_.torque.data());
    return;
  default:
    RCLCPP_ERROR(node_->get_logger(), "Unknown EClientCommandMode provided.");
    return;
  }
}

void LBRClient::init_lbr_command_() {
  std::memcpy(lbr_command_.joint_position.data(), robotState().getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  lbr_command_.torque.fill(0.);
  lbr_command_.wrench.fill(0.);
}

void LBRClient::init_topics_() {
  if (!lbr_state_pub_) {
    auto pub_options = rclcpp::PublisherOptions();
    pub_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &) {
      missed_deadlines_pub_++;
    };

    lbr_state_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRState>(
        robot_name_ + "/state", rclcpp::QoS(1)
                                    .deadline(std::chrono::milliseconds(
                                        static_cast<int64_t>(robotState().getSampleTime() * 1e3)))
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
  }

  if (!lbr_command_sub_) {
    auto memory_strategy =
        rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<
            lbr_fri_msgs::msg::LBRCommand, 1>::make_shared();

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &) {
      missed_deadlines_sub_++;
    };

    lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
        robot_name_ + "/command",
        rclcpp::QoS(1)
            .deadline(
                std::chrono::milliseconds(static_cast<int64_t>(robotState().getSampleTime() * 1e3)))
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
        std::bind(&LBRClient::on_lbr_command_, this, std::placeholders::_1), sub_options,
        memory_strategy);
  }
}

void LBRClient::declare_parameters_() {
  if (!node_->has_parameter("robot_name")) {
    node_->declare_parameter<std::string>("robot_name", "lbr");
  }
  if (!node_->has_parameter("smoothing")) {
    node_->declare_parameter<double>("smoothing", 0.99);
  }
  if (!node_->has_parameter("open_loop")) {
    node_->declare_parameter<bool>("open_loop", true);
  }
}

void LBRClient::get_parameters_() {
  robot_name_ = node_->get_parameter("robot_name").as_string();
  smoothing_ = node_->get_parameter("smoothing").as_double();
  open_loop_ = node_->get_parameter("open_loop").as_bool();
}

void LBRClient::pub_lbr_state_() {
  lbr_state_.client_command_mode = robotState().getClientCommandMode();
  std::memcpy(lbr_state_.commanded_joint_position.data(), robotState().getCommandedJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  std::memcpy(lbr_state_.commanded_torque.data(), robotState().getCommandedTorque(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  lbr_state_.connection_quality = robotState().getConnectionQuality();
  lbr_state_.control_mode = robotState().getControlMode();
  lbr_state_.drive_state = robotState().getDriveState();
  std::memcpy(lbr_state_.external_torque.data(), robotState().getExternalTorque(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    std::memcpy(lbr_state_.ipo_joint_position.data(), robotState().getIpoJointPosition(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }
  if (open_loop_) {
    std::memcpy(lbr_state_.measured_joint_position.data(), robotState().getMeasuredJointPosition(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  } else {

    std::memcpy(lbr_state_.measured_joint_position.data(), lbr_command_.joint_position.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }
  std::memcpy(lbr_state_.measured_torque.data(), robotState().getMeasuredTorque(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
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

void LBRClient::on_lbr_command_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    lbr_command_.joint_position[i] = filters::exponentialSmoothing(
        lbr_command_.joint_position[i], lbr_command->joint_position[i], smoothing_);
  }
  lbr_command_.torque = lbr_command->torque;
  lbr_command_.wrench = lbr_command->wrench;
}
} // end of namespace lbr_fri_ros2
