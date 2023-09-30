#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const rclcpp::Node::SharedPtr node)
    : App(node->get_node_logging_interface(), node->get_node_parameters_interface()) {}

App::App(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
         const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr)
    : logging_interface_ptr_(logging_interface_ptr),
      parameters_interface_ptr_(parameters_interface_ptr), running_(false), run_thread_(nullptr),
      client_(nullptr), connection_(nullptr), app_(nullptr){};

App::~App() {
  stop_run();
  close_udp_socket();
}

bool App::initialize(const std::shared_ptr<KUKA::FRI::LBRClient> &client) {
  if (!client) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "No client provided.");
    return false;
  }
  if (client_ != nullptr) {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "Client already configured.");
    return false;
  }
  if (connection_ != nullptr) {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "Connection already configured.");
    return false;
  }
  if (app_ != nullptr) {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "App already configured.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Initializing LBR FRI ROS 2 App.");
  client_ = client;
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *client_);
  return true;
}

bool App::open_udp_socket(const int &port_id, const char *const remote_host) {
  if (!connection_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Opening UDP socket with port_id %d.", port_id);
  if (!valid_port_(port_id)) {
    return false;
  }
  if (connection_->isOpen()) {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket already open.");
    return true;
  }
  if (!connection_->open(port_id, remote_host)) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Failed to open socket.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket opened successfully.");
  return true;
}

bool App::close_udp_socket() {
  if (!connection_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Closing UDP socket.");
  if (!connection_->isOpen()) {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket already closed.");
    return true;
  }
  connection_->close();
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket closed successfully.");
  return true;
}

void App::run(int rt_prio) {
  if (!client_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Client not configured.");
    return;
  }
  if (!connection_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return;
  }
  if (!connection_->isOpen()) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not open.");
    return;
  }
  if (!app_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "App not configured.");
    return;
  }
  if (run_thread_ != nullptr || running_) {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "App already running.");
    return;
  }
  run_thread_ = std::make_unique<std::thread>([&]() {
    if (realtime_tools::has_realtime_kernel()) {
      if (!realtime_tools::configure_sched_fifo(rt_prio)) {
        RCLCPP_WARN(logging_interface_ptr_->get_logger(),
                    "Failed to set FIFO realtime scheduling policy.");
      }
    } else {
      RCLCPP_WARN(logging_interface_ptr_->get_logger(),
                  "Realtime kernel recommended but not required.");
    }

    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Starting run thread.");
    running_ = true;
    bool success = true;
    while (rclcpp::ok() && success && running_) {
      success = app_->step();
      if (client_->robotState().getSessionState() == KUKA::FRI::ESessionState::IDLE) {
        RCLCPP_INFO(logging_interface_ptr_->get_logger(), "LBR in session state idle, exiting.");
        break;
      }
    }
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Exiting run thread.");
  });
}

void App::stop_run() {
  if (run_thread_ != nullptr) {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Requesting run thread stop.");
    running_ = false;
    run_thread_->join();
    run_thread_ = nullptr;
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Joined run thread.");
  }
}

bool App::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(),
                 "Expected port_id in [30200, 30209], got %d.", port_id);
    return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
