#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
Client::Client(const rclcpp::Node::SharedPtr node, std::unique_ptr<CommandGuard> lbr_command_guard)
    : node_(node), lbr_command_guard_(std::move(lbr_command_guard)),
      external_torque_filter_(node, "external_torque"),
      measured_torque_filter_(node, "measured_torque"),
      joint_position_pid_(node, {"A1", "A2", "A3", "A4", "A5", "A6", "A7"}), topics_init_(false),
      filters_init_(false) {
  missed_deadlines_pub_ = 0;
  missed_deadlines_sub_ = 0;

  declare_parameters_();
  get_parameters_();
}

void Client::log_status() {
  RCLCPP_INFO(node_->get_logger(), "Client - Publisher missed deadlines: %u",
              missed_deadlines_pub_);
  RCLCPP_INFO(node_->get_logger(), "Client - Subscription missed deadlines: %u",
              missed_deadlines_sub_);
}

void Client::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state) {
  init_topics_();
  init_filters_();
  RCLCPP_INFO(node_->get_logger(), "Switched from %s to %s.", KUKA_FRI_STATE_MAP[old_state].c_str(),
              KUKA_FRI_STATE_MAP[new_state].c_str());
}
void Client::monitor() {
  pub_lbr_state_();
  init_lbr_command_();
}

void Client::waitForCommand() {
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

void Client::command() {
  pub_lbr_state_();

  // compute command
  joint_position_pid_.compute(lbr_command_target_.joint_position,
                              lbr_state_.measured_joint_position,
                              rclcpp::Duration(std::chrono::milliseconds(
                                  static_cast<int64_t>(robotState().getSampleTime() * 1e3))),
                              lbr_command_.joint_position);
  lbr_command_.wrench = lbr_command_target_.wrench;
  lbr_command_.torque = lbr_command_target_.torque;

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

void Client::init_lbr_command_() {
  std::memcpy(lbr_command_target_.joint_position.data(), robotState().getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  lbr_command_target_.torque.fill(0.);
  lbr_command_target_.wrench.fill(0.);
  lbr_command_ = lbr_command_target_;
}

void Client::init_topics_() {
  if (topics_init_) {
    return;
  }

  auto pub_options = rclcpp::PublisherOptions();
  pub_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &) {
    missed_deadlines_pub_++;
  };

  lbr_state_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRState>(
      "state", rclcpp::QoS(1)
                   .deadline(std::chrono::milliseconds(
                       static_cast<int64_t>(robotState().getSampleTime() * 1e3)))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));

  auto memory_strategy =
      rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<
          lbr_fri_msgs::msg::LBRCommand, 1>::make_shared();

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &) {
    missed_deadlines_sub_++;
  };

  lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
      "command",
      rclcpp::QoS(1)
          .deadline(
              std::chrono::milliseconds(static_cast<int64_t>(robotState().getSampleTime() * 1e3)))
          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
      std::bind(&Client::on_lbr_command_, this, std::placeholders::_1), sub_options,
      memory_strategy);

  topics_init_ = true;
}

void Client::init_filters_() {
  if (filters_init_) {
    return;
  }
  // initialize PID
  joint_position_pid_.init(2. * robotState().getSampleTime(), 0., 0., 0., 0., false);

  // initialize torque filters
  external_torque_filter_.init(10., robotState().getSampleTime());
  measured_torque_filter_.init(10., robotState().getSampleTime());

  filters_init_ = true;
}

void Client::declare_parameters_() {
  if (!node_->has_parameter("open_loop")) {
    node_->declare_parameter<bool>("open_loop", true);
  }
}

void Client::get_parameters_() { open_loop_ = node_->get_parameter("open_loop").as_bool(); }

void Client::pub_lbr_state_() {
  lbr_state_.client_command_mode = robotState().getClientCommandMode();
  std::memcpy(lbr_state_.commanded_joint_position.data(), robotState().getCommandedJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  std::memcpy(lbr_state_.commanded_torque.data(), robotState().getCommandedTorque(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  lbr_state_.connection_quality = robotState().getConnectionQuality();
  lbr_state_.control_mode = robotState().getControlMode();
  lbr_state_.drive_state = robotState().getDriveState();
  external_torque_filter_.compute(robotState().getExternalTorque(), lbr_state_.external_torque);
  if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    std::memcpy(lbr_state_.ipo_joint_position.data(), robotState().getIpoJointPosition(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }
  if (open_loop_) {
    std::memcpy(lbr_state_.measured_joint_position.data(), lbr_command_.joint_position.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  } else {
    std::memcpy(lbr_state_.measured_joint_position.data(), robotState().getMeasuredJointPosition(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }
  measured_torque_filter_.compute(robotState().getMeasuredTorque(), lbr_state_.measured_torque);
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

void Client::on_lbr_command_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  lbr_command_target_ = *lbr_command;
}
} // end of namespace lbr_fri_ros2
