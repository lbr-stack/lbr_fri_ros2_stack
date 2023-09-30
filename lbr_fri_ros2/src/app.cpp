#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const rclcpp::Node::SharedPtr node)
    : App(node->get_node_logging_interface(), node->get_node_parameters_interface()) {}

App::App(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr) {
  connected_ = false;

  // factory method for creating a client
  client_ = client_factory(logging_interface_ptr, parameters_interface_ptr, "position");
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *client_);
}

App::~App() { disconnect(); }

bool App::connect(const int &port_id, const char *const remote_host) {
  RCLCPP_INFO(logging_interface_ptr_->get_logger(),
              "Attempting to open UDP socket with port_id %d for LBR server...", port_id);
}

bool App::disconnect() {
  RCLCPP_INFO(logging_interface_ptr_->get_logger(),
              "Attempting to close UDP socket for LBR server...");
}

void App::run(int rt_prio) {
  if (realtime_tools::has_realtime_kernel()) {
    if (!realtime_tools::configure_sched_fifo(rt_prio)) {
      RCLCPP_WARN(logging_interface_ptr_->get_logger(),
                  "Failed to set FIFO realtime scheduling policy.");
    }
  } else {
    RCLCPP_WARN(logging_interface_ptr_->get_logger(), "Realtime kernel recommended.");
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
