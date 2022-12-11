#include "lbr_fri_ros2/lbr_app_node.hpp"

namespace lbr_fri_ros2 {
LBRAppNode::LBRAppNode(const std::string &node_name, const int &port_id,
                       const char *const remote_host)
    : rclcpp::Node(node_name) {
  if (!valid_port_(port_id)) {
    throw std::range_error("Invalid port_id provided.");
  }
  port_id_ = port_id;
  remote_host_ = remote_host;

  connected_ = false;

  declare_parameter("joint_velocity_command_limit", std::vector<double>(LBR::JOINT_DOF, 0.));
  declare_parameter("wrench_command_limit", std::vector<double>(LBR::CARTESIAN_DOF, 0.));
  declare_parameter("torque_command_limit", std::vector<double>(LBR::JOINT_DOF, 0.));

  joint_velocity_command_limit_ = get_parameter("joint_velocity_command_limit").as_double_array();
  wrench_command_limit_ = get_parameter("wrench_command_limit").as_double_array();
  torque_command_limit_ = get_parameter("torque_command_limit").as_double_array();

  app_connect_srv_ = create_service<lbr_fri_msgs::srv::AppConnect>(
      "/lbr_app/connect",
      std::bind(&LBRAppNode::app_connect_cb_, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_system_default);

  app_disconnect_srv_ = create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "/lbr_app/disconnect",
      std::bind(&LBRAppNode::app_disconnect_cb_, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_system_default);

  lbr_command_rt_buf_ =
      std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>(
          nullptr);
  lbr_command_sub_ = create_subscription<lbr_fri_msgs::msg::LBRCommand>(
      "/lbr_command", rclcpp::SystemDefaultsQoS(),
      std::bind(&LBRAppNode::lbr_command_sub_cb_, this, std::placeholders::_1));
  lbr_state_pub_ =
      create_publisher<lbr_fri_msgs::msg::LBRState>("/lbr_state", rclcpp::SystemDefaultsQoS());
  lbr_state_rt_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(
          lbr_state_pub_);

  lbr_ = std::make_shared<LBR>();
  lbr_client_ = std::make_shared<LBRClient>(lbr_);
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *lbr_client_);

  // attempt default connect
  connect_(port_id_, remote_host_);
}

LBRAppNode::~LBRAppNode() { disconnect_(); }

void LBRAppNode::app_connect_cb_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                                 lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  const char *remote_host = request->remote_host.empty() ? NULL : request->remote_host.c_str();
  try {
    response->connected = connect_(request->port_id, remote_host);
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(get_logger(), "Failed. %s", e.what());
  }
}

void LBRAppNode::app_disconnect_cb_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  try {
    response->disconnected = disconnect_();
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(get_logger(), "Failed. %s", e.what());
  }
}

void LBRAppNode::lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  lbr_command_rt_buf_->writeFromNonRT(lbr_command);
}

bool LBRAppNode::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(get_logger(), "Expected port_id id in [30200, 30209], got %d.", port_id);
    return false;
  }
  return true;
}

bool LBRAppNode::connect_(const int &port_id, const char *const remote_host) {
  RCLCPP_INFO(get_logger(), "Attempting to open UDP socket for LBR server...");
  if (!connected_) {
    if (!valid_port_(port_id)) {
      throw std::range_error("Invalid port_id provided.");
    }
    connected_ = app_->connect(port_id, remote_host);
    if (connected_) {
      port_id_ = port_id;
      remote_host_ = remote_host;

      auto app_step = [this]() {
        bool success = true;
        while (success && connected_ && rclcpp::ok()) {
          try {
            auto lbr_command = lbr_command_rt_buf_->readFromRT();
            if (valid_lbr_command_(lbr_command)) {
              lbr_->command = *lbr_command;
            }
            success = app_->step();
            if (lbr_state_rt_pub_->trylock()) {
              lbr_state_rt_pub_->msg_ = *lbr_->state;
              lbr_state_rt_pub_->unlockAndPublish();
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), e.what());
            break;
          }
        }
        if (connected_) {
          disconnect_();
        }
      };

      app_step_thread_ = std::make_unique<std::thread>(app_step);
    }
  } else {
    RCLCPP_INFO(get_logger(), "Port already open.");
  }
  if (connected_) {
    RCLCPP_INFO(get_logger(), "Opened successfully.");
  } else {
    RCLCPP_WARN(get_logger(), "Failed to open.");
  }
  return connected_;
}

bool LBRAppNode::disconnect_() {
  RCLCPP_INFO(get_logger(), "Attempting to close UDP socket for LBR server...");
  if (connected_) {
    app_->disconnect();
    connected_ = false;
  } else {
    RCLCPP_INFO(get_logger(), "Port already closed.");
  }
  if (!connected_) {
    RCLCPP_INFO(get_logger(), "Closed successfully.");
  } else {
    RCLCPP_WARN(get_logger(), "Failed to close.");
  }
  return !connected_;
}

bool LBRAppNode::valid_lbr_command_(
    const lbr_fri_msgs::msg::LBRCommand::SharedPtr *const lbr_command) {
  if (!lbr_command || !(*lbr_command)) {
    return false;
  }

  switch (lbr_->state->client_command_mode) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE: {
    return true;
  }
  case KUKA::FRI::EClientCommandMode::POSITION: {
    return valid_joint_position_command_((*lbr_command)->joint_position);
  }
  case KUKA::FRI::EClientCommandMode::WRENCH: {
    return valid_joint_position_command_((*lbr_command)->joint_position) &&
           valid_wrench_command_((*lbr_command)->wrench);
  }
  case KUKA::FRI::EClientCommandMode::TORQUE: {
    return valid_joint_position_command_((*lbr_command)->joint_position) &&
           valid_torque_command_((*lbr_command)->torque);
  }
  default: {
    RCLCPP_ERROR(get_logger(), "Unknown EClientCommandMode provided.");
    break;
  }
  }
  return false;
}

bool LBRAppNode::valid_joint_position_command_(const std::vector<double> &joint_position_command) {
  if (joint_position_command.size() != LBR::JOINT_DOF) {
    RCLCPP_WARN(get_logger(), "Joint position command of size %lu received. Expected size %d.",
                joint_position_command.size(), LBR::JOINT_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::JOINT_DOF; ++i) {
    if (std::abs(joint_position_command[i] - lbr_->state->measured_joint_position[i]) >
        joint_velocity_command_limit_[i] * lbr_->state->sample_time) {
      RCLCPP_WARN(get_logger(), "Got joint velocity command abs(%f) on joint %d. Limit is %f.",
                  std::abs(joint_position_command[i] - lbr_->state->measured_joint_position[i]) /
                      lbr_->state->sample_time,
                  i, joint_velocity_command_limit_[i]);
      return false;
    }
  }
  return true;
}

bool LBRAppNode::valid_wrench_command_(const std::vector<double> &wrench_command) {
  if (wrench_command.size() != LBR::CARTESIAN_DOF) {
    RCLCPP_WARN(get_logger(), "Wrench command of size %lu received. Expected size %d.",
                wrench_command.size(), LBR::CARTESIAN_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::CARTESIAN_DOF; ++i) {
    if (std::abs(wrench_command[i]) > wrench_command_limit_[i]) {
      RCLCPP_WARN(get_logger(), "Got wrench command abs(%f) on axis %d. Limit is %f.",
                  std::abs(wrench_command[i]), i, wrench_command_limit_[i]);
      return false;
    }
  }
  return true;
}

bool LBRAppNode::valid_torque_command_(const std::vector<double> &torque_command) {
  if (torque_command.size() != LBR::JOINT_DOF) {
    RCLCPP_WARN(get_logger(), "Torque command of size %lu received. Expected size %d.",
                torque_command.size(), LBR::JOINT_DOF);
    return false;
  }
  for (uint8_t i = 0; i < LBR::JOINT_DOF; ++i) {
    if (std::abs(torque_command[i]) > torque_command_limit_[i]) {
      RCLCPP_WARN(get_logger(), "Got torque command abs(%f) on joint %d. Limit is %f.",
                  std::abs(torque_command[i]), i, torque_command_limit_[i]);
      return false;
    }
  }
  return true;
}
} // end of namespace lbr_fri_ros2
