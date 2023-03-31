#ifndef LBR_FRI_ROS2__LBR_APP_HPP_
#define LBR_FRI_ROS2__LBR_APP_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "fri/friClientApplication.h"
#include "fri/friClientIf.h"
#include "fri/friUdpConnection.h"

#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/lbr_client.hpp"
#include "lbr_fri_ros2/lbr_command_guard.hpp"
#include "lbr_fri_ros2/lbr_intermediary.hpp"

namespace lbr_fri_ros2 {
class LBRApp : public rclcpp::Node {
public:
  LBRApp(const rclcpp::NodeOptions &options);
  ~LBRApp();

protected:
  void declare_parameters_();
  void get_parameters_();

  void app_connect_cb_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                       lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

  void app_disconnect_cb_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
                          lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

  void lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);

  bool valid_port_(const int &port_id);
  bool connect_(const int &port_id = 30200, const char *const remote_host = NULL);
  bool disconnect_();

  void step_();

  std::unique_ptr<std::thread> app_step_thread_;

  const char *remote_host_;
  int port_id_;

  std::atomic<bool> connected_;

  rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_srv_;
  rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_srv_;

  std::shared_ptr<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>
      lbr_command_rt_buf_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>> lbr_state_rt_pub_;

  std::shared_ptr<LBRIntermediary> lbr_intermediary_;
  std::shared_ptr<LBRClient> lbr_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_APP_HPP_
