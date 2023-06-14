#include "lbr_fri_ros2/lbr_app.hpp"

namespace lbr_fri_ros2 {
LBRApp::LBRApp(const rclcpp::Node::SharedPtr node) : node_(node) {
  declare_parameters_();
  get_parameters_();

  connected_ = false;

  app_connect_srv_ = node_->create_service<lbr_fri_msgs::srv::AppConnect>(
      "~/connect",
      std::bind(&LBRApp::app_connect_cb_, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default);

  app_disconnect_srv_ = node_->create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "~/disconnect",
      std::bind(&LBRApp::app_disconnect_cb_, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default);

  std::string robot_description;
  node_->declare_parameter<std::string>("robot_description");
  if (!node_->get_parameter("robot_description", robot_description)) {
    throw std::runtime_error("Failed to receive robot_description parameter.");
  }

  lbr_client_ =
      std::make_shared<LBRClient>(node_, lbr_fri_ros2::LBREarlyStopCommandGuard{robot_description});
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *lbr_client_);

  // attempt default connect
  connect_(port_id_, remote_host_);
}

LBRApp::~LBRApp() { disconnect_(); }

void LBRApp::declare_parameters_() {
  node_->declare_parameter<int>("port_id", 30200);
  node_->declare_parameter<std::string>("remote_host", "");
}

void LBRApp::get_parameters_() {
  int port_id = node_->get_parameter("port_id").as_int();
  std::string remote_host = node_->get_parameter("remote_host").as_string();

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
    RCLCPP_ERROR(node_->get_logger(), "Failed. %s", e.what());
  }
}

void LBRApp::app_disconnect_cb_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  try {
    response->disconnected = disconnect_();
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed. %s", e.what());
  }
}

bool LBRApp::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(node_->get_logger(), "Expected port_id in [30200, 30209], got %d.", port_id);
    return false;
  }
  return true;
}

bool LBRApp::connect_(const int &port_id, const char *const remote_host) {
  RCLCPP_INFO(node_->get_logger(),
              "Attempting to open UDP socket with port_id %d for LBR server...", port_id);
  if (!connected_) {
    if (!valid_port_(port_id)) {
      throw std::range_error("Invalid port_id provided.");
    }
    connected_ = app_->connect(port_id, remote_host);
    if (connected_) {
      port_id_ = port_id;
      remote_host_ = remote_host;
      run_thread_ = std::make_unique<std::thread>(std::bind(&LBRApp::run_, this));
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Port already open.");
  }
  if (connected_) {
    RCLCPP_INFO(node_->get_logger(), "Opened successfully.");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to open.");
  }
  return connected_;
}

bool LBRApp::disconnect_() {
  RCLCPP_INFO(node_->get_logger(),
              "Attempting to close UDP socket with port_id %d for LBR server...", port_id_);
  if (connected_) {
    app_->disconnect();
    connected_ = false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Port already closed.");
  }
  if (!connected_) {
    RCLCPP_INFO(node_->get_logger(), "Closed successfully.");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to close.");
  }
  auto future = std::async(std::launch::async, &std::thread::join, run_thread_.get());
  if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    throw std::runtime_error("Could not join app step thread.");
  }
  run_thread_.release();
  return !connected_;
}

void LBRApp::run_() {
  bool success = true;
  while (success && connected_ && rclcpp::ok()) {
    success = app_->step();
    if (lbr_client_->robotState().getSessionState() == KUKA::FRI::IDLE) {
      break;
    }
  }
}
} // end of namespace lbr_fri_ros2
