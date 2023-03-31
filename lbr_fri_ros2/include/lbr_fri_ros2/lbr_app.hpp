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
/**
 * @brief Node for exposing FRI commands / states to realtime-safe topics.
 *
 * Robot commands of type lbr_fri_msgs::msg::LBRCommand are subscribed to from
 * <b>/lbr_command</b>. Robot states of type lbr_fri_msgs::msg::LBRState are published to
 * <b>/lbr_state</b>.
 *
 * Commands
 *
 */
class LBRApp : public rclcpp::Node {
public:
  LBRApp(const rclcpp::NodeOptions &options);
  ~LBRApp();

protected:
  /**
   * @brief Declares parameters for this node.
   *
   */
  void declare_parameters_();

  /**
   * @brief Gets parameters for this node and writes them into members. Checks validity.
   *
   * @throws std::runtime_error for invalid parameters
   */
  void get_parameters_();

  /**
   * @brief Callback to <b>~/connect</b> service. Calls #connect_.
   *
   * @param[in] request Request containing port_id and remote_host
   * @param[out] response Response containing connected and message
   */
  void app_connect_cb_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                       lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

  /**
   * @brief Callback to <b>~/disconnect</b> service. Calls #disconnect_.
   *
   * @param[in] request Empty request
   * @param[out] response Response containing disconnected and message
   */
  void app_disconnect_cb_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
                          lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

  /**
   * @brief Callback to <b>/lbr_command</b> topic. Writes command into #lbr_command_rt_buf_
   * buffer.
   *
   * @param[in] lbr_command Command of type lbr_fri_msgs::msg::LBRCommand
   */
  void lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);

  /**
   * @brief Checks for valid port id.
   *
   * @param[in] port_id The port id
   * @return true if port id in [30200, 30209]
   * @return false else
   */
  bool valid_port_(const int &port_id);

  /**
   * @brief Opens a UDP port and spawns the #app_step_thread_.
   *
   * @param[in] port_id The port id, allowed values [30200, 30209]
   * @param[in] remote_host The address of the remote host
   * @return true if successfully opened / already open
   * @return false if failed to open
   */
  bool connect_(const int &port_id = 30200, const char *const remote_host = NULL);

  /**
   * @brief Closes the UDP port and joins the #app_step_thread_.
   *
   * @return true if closed successfully / already closed
   * @return false if failed to close
   */
  bool disconnect_();

  /**
   * @brief Reads commands from <b>/lbr_command</b> and writes them into #lbr_intermediary_.
   * Calls #app_::step(), which callbacks #lbr_client_. #lbr_client_ shares #lbr_intermediary_ and
   *
   */
  void step_();

  std::unique_ptr<std::thread> app_step_thread_; /**< Thread running the #step_ method*/

  const char *remote_host_; /**< The remote host's IP address*/
  int port_id_;             /**< The UDP port id*/

  std::atomic<bool> connected_; /**< True if UDP port open and communication running*/

  rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_srv_;
  rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_srv_;

  std::shared_ptr<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>
      lbr_command_rt_buf_; /**< Realtime buffer for receiving lbr_fri_msgs::msg::LBRCommand
                              commands*/
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>> lbr_state_rt_pub_;

  std::shared_ptr<LBRIntermediary>
      lbr_intermediary_; /**< Shared lbr_fri_ros2::LBRIntermediary object with #lbr_client_*/
  std::shared_ptr<LBRClient>
      lbr_client_; /**< Writes commands / reads states from #lbr_intermediary_ to robot*/
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_APP_HPP_
