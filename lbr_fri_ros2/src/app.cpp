#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const rclcpp::Node::SharedPtr node) : node_(node) {
  declare_parameters_();
  get_parameters_();

  connected_ = false;

  app_connect_srv_ = node_->create_service<lbr_fri_msgs::srv::AppConnect>(
      "connect",
      std::bind(&App::on_app_connect_, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default);

  app_disconnect_srv_ = node_->create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "disconnect",
      std::bind(&App::on_app_disconnect_, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default);

  client_ = std::make_shared<Client>(
      node_, lbr_command_guard_factory(node_->get_node_logging_interface(), robot_description_,
                                       command_guard_variant_));
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *client_);

  // attempt default connect
  connect_(port_id_, remote_host_);
}

App::~App() { disconnect_(); }

void App::declare_parameters_() {
  if (!node_->has_parameter("port_id")) {
    node_->declare_parameter<int>("port_id", 30200);
  }
  if (!node_->has_parameter("remote_host")) {
    node_->declare_parameter<std::string>("remote_host", "");
  }
  if (!node_->has_parameter("robot_description")) {
    node_->declare_parameter<std::string>("robot_description", "");
  }
  if (!node_->has_parameter("command_guard_variant")) {
    node_->declare_parameter<std::string>("command_guard_variant", "safe_stop");
  }
  if (!node_->has_parameter("rt_prio")) {
    node_->declare_parameter<int>("rt_prio", 80);
  }
}

void App::get_parameters_() {
  int port_id = node_->get_parameter("port_id").as_int();
  std::string remote_host = node_->get_parameter("remote_host").as_string();

  if (!valid_port_(port_id)) {
    throw std::range_error("Invalid port_id provided.");
  }
  port_id_ = port_id;
  remote_host_ = remote_host.empty() ? NULL : remote_host.c_str();

  if (!node_->get_parameter("robot_description", robot_description_)) {
    throw std::runtime_error("Failed to receive robot_description parameter.");
  }
  command_guard_variant_ = node_->get_parameter("command_guard_variant").as_string();
  rt_prio_ = node_->get_parameter("rt_prio").as_int();
}

void App::on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                          lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  const char *remote_host = request->remote_host.empty() ? NULL : request->remote_host.c_str();
  try {
    response->connected = connect_(request->port_id, remote_host);
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed. %s", e.what());
  }
}

void App::on_app_disconnect_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
                             lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  try {
    response->disconnected = disconnect_();
  } catch (const std::exception &e) {
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed. %s", e.what());
  }
}

bool App::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(node_->get_logger(), "Expected port_id in [30200, 30209], got %d.", port_id);
    return false;
  }
  return true;
}

bool App::connect_(const int &port_id, const char *const remote_host) {
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
      run_thread_ = std::make_unique<std::thread>(std::bind(&App::run_, this));
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

bool App::disconnect_() {
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

void App::run_() {
  if (realtime_tools::has_realtime_kernel()) {
    if (!realtime_tools::configure_sched_fifo(rt_prio_)) {
      RCLCPP_WARN(node_->get_logger(), "Failed to set FIFO realtime scheduling policy.");
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Realtime kernel recommended.");
  }

  bool success = true;
  while (success && connected_ && rclcpp::ok()) {
    success = app_->step();
    if (client_->robotState().getSessionState() == KUKA::FRI::IDLE) {
      break;
    }
  }
}
} // end of namespace lbr_fri_ros2
