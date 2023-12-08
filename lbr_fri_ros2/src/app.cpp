#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const std::shared_ptr<AsyncClient> async_client_ptr)
    : should_stop_(true), running_(false), async_client_ptr_(nullptr), connection_ptr_(nullptr),
      app_ptr_(nullptr) {
  async_client_ptr_ = async_client_ptr;
  connection_ptr_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ptr_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_ptr_, *async_client_ptr_);
}

App::~App() {
  stop_run();
  close_udp_socket();
}

bool App::open_udp_socket(const int &port_id, const char *const remote_host) {
  if (!connection_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Opening UDP socket with port_id %d.", port_id);
  if (!valid_port_(port_id)) {
    return false;
  }
  if (connection_ptr_->isOpen()) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Socket already open.");
    return true;
  }
  if (!connection_ptr_->open(port_id, remote_host)) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to open socket.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Socket opened successfully.");
  return true;
}

bool App::close_udp_socket() {
  if (!connection_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Connection not configured.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Closing UDP socket.");
  if (!connection_ptr_->isOpen()) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Socket already closed.");
    return true;
  }
  connection_ptr_->close();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Socket closed successfully.");
  return true;
}

void App::run(int rt_prio) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "AsyncClient not configured.");
    return;
  }
  if (!connection_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Connection not configured.");
    return;
  }
  if (!connection_ptr_->isOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Connection not open.");
    return;
  }
  if (!app_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "App not configured.");
    return;
  }
  if (running_) {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "App already running.");
    return;
  }
  run_thread_ = std::thread([&]() {
    if (realtime_tools::has_realtime_kernel()) {
      if (!realtime_tools::configure_sched_fifo(rt_prio)) {
        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                    "Failed to set FIFO realtime scheduling policy.");
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Realtime kernel recommended but not required.");
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting run thread.");
    should_stop_ = false;
    running_ = true;
    bool success = true;
    while (rclcpp::ok() && success && !should_stop_) {
      success = app_ptr_->step(); // TODO: blocks until robot heartbeat, stuck if port id mismatches
      if (async_client_ptr_->robotState().getSessionState() == KUKA::FRI::ESessionState::IDLE) {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "LBR in session state idle, exiting.");
        break;
      }
    }
    async_client_ptr_->get_state_interface().uninitialize();
    running_ = false;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Exiting run thread.");
  });
  run_thread_.detach();
}

void App::stop_run() {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Requesting run thread stop.");
  should_stop_ = true;
}

bool App::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Expected port_id in [30200, 30209], got %d.",
                 port_id);
    return false;
  }
  return true;
}
} // end of namespace lbr_fri_ros2
