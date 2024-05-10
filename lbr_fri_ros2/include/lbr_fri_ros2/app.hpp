#ifndef LBR_FRI_ROS2__APP_HPP_
#define LBR_FRI_ROS2__APP_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/thread_priority.hpp"

#include "friClientApplication.h"
#include "friUdpConnection.h"

#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/formatting.hpp"

namespace lbr_fri_ros2 {
class App {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::App";

public:
  App(const std::shared_ptr<AsyncClient> async_client_ptr);
  ~App();

  bool open_udp_socket(const int &port_id = 30200, const char *const remote_host = NULL);
  bool close_udp_socket();
  void run_async(int rt_prio = 80);
  void request_stop();

protected:
  bool valid_port_(const int &port_id);

  std::atomic_bool should_stop_, running_;
  std::thread run_thread_;

  std::shared_ptr<AsyncClient> async_client_ptr_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_ptr_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_ptr_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__APP_HPP_
