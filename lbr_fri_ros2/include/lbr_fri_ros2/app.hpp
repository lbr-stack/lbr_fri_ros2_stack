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
#include "lbr_fri_ros2/worker.hpp"

namespace lbr_fri_ros2 {
class App : public Worker {
  /**
   * @brief This clas provides utilities to run the KUKA::FRI::ClientApplication asynchronously.
   * Note that the rate at which the application runs is determined by the robot. This is because
   * the run_thread_ uses blocking function calls from the FRI client SDK, i.e.
   * KUKA::FRI::ClientApplication::step() (this is by KUKA's design).
   *
   */
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::App";

public:
  App(const std::shared_ptr<AsyncClient> async_client_ptr);
  ~App();

  bool open_udp_socket(const int &port_id = 30200, const char *const remote_host = NULL);
  bool close_udp_socket();
  void run_async(int rt_prio = 80) override;

protected:
  void perform_work_() override;
  bool valid_port_(const int &port_id);

  std::atomic_bool should_stop_, running_;
  std::thread run_thread_;

  std::shared_ptr<AsyncClient> async_client_ptr_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_ptr_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_ptr_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__APP_HPP_
