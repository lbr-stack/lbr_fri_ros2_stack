#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const rclcpp::Node::SharedPtr node_ptr, const std::shared_ptr<Client> client_ptr)
    : logging_interface_ptr_(node_ptr->get_node_logging_interface()),
      parameters_interface_ptr_(node_ptr->get_node_parameters_interface()), should_stop_(true),
      running_(false), client_ptr_(nullptr), connection_ptr_(nullptr), app_ptr_(nullptr) {
  client_ptr_ = client_ptr;
  connection_ptr_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ptr_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_ptr_, *client_ptr_);
}

App::~App() {
  stop_run();
  close_udp_socket();
}

bool App::open_udp_socket(const int &port_id, const char *const remote_host) {
  if (!connection_ptr_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Opening UDP socket with port_id %d.", port_id);
  if (!valid_port_(port_id)) {
    return false;
  }
  if (connection_ptr_->isOpen()) {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket already open.");
    return true;
  }
  if (!connection_ptr_->open(port_id, remote_host)) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Failed to open socket.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket opened successfully.");
  return true;
}

bool App::close_udp_socket() {
  if (!connection_ptr_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Closing UDP socket.");
  if (!connection_ptr_->isOpen()) {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket already closed.");
    return true;
  }
  connection_ptr_->close();
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Socket closed successfully.");
  return true;
}

void App::run(int rt_prio) {
  if (!client_ptr_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Client not configured.");
    return;
  }
  if (!connection_ptr_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not configured.");
    return;
  }
  if (!connection_ptr_->isOpen()) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Connection not open.");
    return;
  }
  if (!app_ptr_) {
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "App not configured.");
    return;
  }
  if (running_) {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "App already running.");
    return;
  }
  run_thread_ = std::thread([&]() {
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
    should_stop_ = false;
    running_ = true;
    bool success = true;
    while (rclcpp::ok() && success && !should_stop_) {
      success = app_ptr_->step(); // TODO: blocks until robot heartbeat, stuck if port id mismatches
      if (client_ptr_->robotState().getSessionState() == KUKA::FRI::ESessionState::IDLE) {
        RCLCPP_INFO(logging_interface_ptr_->get_logger(), "LBR in session state idle, exiting.");
        break;
      }
    }
    client_ptr_->get_state_interface().uninitialize();
    running_ = false;
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Exiting run thread.");
  });
  run_thread_.detach();
}

void App::stop_run() {
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Requesting run thread stop.");
  should_stop_ = true;
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
