#include "lbr_fri_ros2/lbr_app.hpp"

namespace lbr_fri_ros2 {
LBRApp::LBRApp(const rclcpp::NodeOptions &options) : rclcpp::Node("lbr_app", options) {
  declare_parameters_();
  get_parameters_();

  connected_ = false;

  app_connect_srv_ = create_service<lbr_fri_msgs::srv::AppConnect>(
      "~/connect",
      std::bind(&LBRAppNode::app_connect_cb_, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS());

  app_disconnect_srv_ = create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "~/disconnect",
      std::bind(&LBRAppNode::app_disconnect_cb_, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS());

  lbr_command_rt_buf_ =
      std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>(
          nullptr);
  lbr_command_sub_ = create_subscription<lbr_fri_msgs::msg::LBRCommand>(
      "/lbr_command", rclcpp::SensorDataQoS(),
      std::bind(&LBRApp::lbr_command_sub_cb_, this, std::placeholders::_1));
  lbr_state_pub_ =
      create_publisher<lbr_fri_msgs::msg::LBRState>("/lbr_state", rclcpp::SensorDataQoS());
  lbr_state_rt_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(
          lbr_state_pub_);

  std::string robot_description;
  declare_parameter<std::string>("robot_description");
  if (!get_parameter("robot_description", robot_description)) {
    throw std::runtime_error("Failed to receive robot_description parameter.");
  }

  lbr_intermediary_ =
      std::make_shared<LBRIntermediary>(LBREarlyStopCommandGuard{robot_description});
  lbr_client_ = std::make_shared<LBRClient>(lbr_intermediary_);
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *lbr_client_);

  // attempt default connect
  connect_(port_id_, remote_host_);
}

LBRApp::~LBRApp() { disconnect_(); }

void LBRApp::declare_parameters_() {
  declare_parameter<int>("port_id", 30200);
  declare_parameter<std::string>("remote_host", "");
}

void LBRApp::get_parameters_() {
  int port_id = get_parameter("port_id").as_int();
  std::string remote_host = get_parameter("remote_host").as_string();

  if (!valid_port_(port_id)) {
    throw std::range_error("Invalid port_id provided.");
  }
  port_id_ = port_id;
  remote_host_ = remote_host.empty() ? NULL : remote_host.c_str();
}

void LBRApp::app_connect_cb_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                             lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  const char *remote_host = request->remote_host.empty() ? NULL : request->remote_host.c_str();
  try {
    response->connected = connect_(request->port_id, remote_host);
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(get_logger(), "Failed. %s", e.what());
  }
}

void LBRApp::app_disconnect_cb_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  try {
    response->disconnected = disconnect_();
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(get_logger(), "Failed. %s", e.what());
  }
}

void LBRApp::lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command) {
  lbr_command_rt_buf_->writeFromNonRT(lbr_command);
}

bool LBRApp::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(get_logger(), "Expected port_id in [30200, 30209], got %d.", port_id);
    return false;
  }
  return true;
}

bool LBRApp::connect_(const int &port_id, const char *const remote_host) {
  RCLCPP_INFO(get_logger(), "Attempting to open UDP socket with port_id %d for LBR server...",
              port_id);
  if (!connected_) {
    if (!valid_port_(port_id)) {
      throw std::range_error("Invalid port_id provided.");
    }
    connected_ = app_->connect(port_id, remote_host);
    if (connected_) {
      port_id_ = port_id;
      remote_host_ = remote_host;
      app_step_thread_ = std::make_unique<std::thread>(std::bind(&LBRApp::step_, this));
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

bool LBRApp::disconnect_() {
  RCLCPP_INFO(get_logger(), "Attempting to close UDP socket with port_id %d for LBR server...",
              port_id_);
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
  auto future = std::async(std::launch::async, &std::thread::join, app_step_thread_.get());
  if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    throw std::runtime_error("Could not join app step thread.");
  }
  app_step_thread_.release();
  return !connected_;
}

void LBRApp::step_() {
  bool success = true;
  while (success && connected_ && rclcpp::ok()) {
    try {
      if (lbr_intermediary_->lbr_state().session_state ==
          KUKA::FRI::ESessionState::COMMANDING_WAIT) {
        lbr_command_rt_buf_->reset();
      }
      auto lbr_command = *lbr_command_rt_buf_->readFromRT();
      lbr_intermediary_->command_to_buffer(lbr_command);
      success = app_->step();
      if (lbr_state_rt_pub_->trylock()) {
        lbr_intermediary_->buffer_to_state(lbr_state_rt_pub_->msg_);
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
}
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::LBRApp);
