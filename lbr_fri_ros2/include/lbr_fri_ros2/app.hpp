#ifndef LBR_FRI_ROS2__APP_HPP_
#define LBR_FRI_ROS2__APP_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

#include "friClientApplication.h"
#include "friUdpConnection.h"

#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
class App {
public:
  App(const rclcpp::Node::SharedPtr node_ptr, const std::shared_ptr<Client> client_ptr);
  ~App();

  bool open_udp_socket(const int &port_id = 30200, const char *const remote_host = NULL);
  bool close_udp_socket();
  void run(int rt_prio = 80);
  void stop_run();

protected:
  bool valid_port_(const int &port_id);

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  std::atomic_bool should_stop_;
  std::unique_ptr<std::thread> run_thread_ptr_;

  std::shared_ptr<Client> client_ptr_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_ptr_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_ptr_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__APP_HPP_
