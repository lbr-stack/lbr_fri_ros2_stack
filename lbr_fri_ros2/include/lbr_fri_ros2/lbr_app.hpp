#ifndef LBR_FRI_ROS2__LBR_APP_HPP_
#define LBR_FRI_ROS2__LBR_APP_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "fri/friClientApplication.h"
#include "fri/friClientIf.h"
#include "fri/friUdpConnection.h"

#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/lbr_client.hpp"
#include "lbr_fri_ros2/lbr_command_guard.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief The LBRApp has a node for exposing FRI methods to services. It shares this node with the
 * #lbr_client_, which reads commands / write states via real-time safe topics.
 *
 * Services:
 * - <b>~/connect</b> (lbr_fri_msgs::srv::AppConnect)
 * Opens UDP port to FRI. Creates #run_thread_ thread via #on_app_connect_ that calls #run_ to
 * communicate with the robot.
 * - <b>~/disconnect</b> (lbr_fri_msgs::srv::AppDisconnect)
 * Closes UDP port to FRI. Finishes #run_thread_ thread via #on_app_disconnect_ through ending
 * #run_.
 *
 */
class LBRApp {
public:
  /**
   * @brief Construct a new LBRApp object.
   *
   * @param node Shared node
   *
   * @throws std::runtime error if no robot_description in node parameters
   */
  LBRApp(const rclcpp::Node::SharedPtr node);
  ~LBRApp();

protected:
  /**
   * @brief Declares parameters for node.
   *
   */
  void declare_parameters_();

  /**
   * @brief Gets parameters for node and writes them into members. Checks validity.
   *
   * @throws std::range_error for invalid port_id in node parameters
   */
  void get_parameters_();

  /**
   * @brief Callback to <b>~/connect</b> service. Calls #connect_.
   *
   * @param[in] request Request containing port_id and remote_host
   * @param[out] response Response containing connected and message
   */
  void on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                       lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

  /**
   * @brief Callback to <b>~/disconnect</b> service. Calls #disconnect_.
   *
   * @param[in] request Empty request
   * @param[out] response Response containing disconnected and message
   */
  void on_app_disconnect_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
                          lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

  /**
   * @brief Checks for valid port id.
   *
   * @param[in] port_id The port id
   * @return true if port id in [30200, 30209]
   * @return false else
   */
  bool valid_port_(const int &port_id);

  /**
   * @brief Opens a UDP port and spawns the #run_thread_.
   *
   * @param[in] port_id The port id, allowed values [30200, 30209]
   * @param[in] remote_host The address of the remote host
   * @return true if successfully opened / already open
   * @return false if failed to open
   *
   * @throws std::range_error if invalid port_id
   *
   */
  bool connect_(const int &port_id = 30200, const char *const remote_host = NULL);

  /**
   * @brief Closes the UDP port and joins the #run_thread_.
   *
   * @return true if closed successfully / already closed
   * @return false if failed to close
   *
   * @throws std::runtime_error if #run_thread_ fails to join
   *
   */
  bool disconnect_();

  /**
   * @brief Exchanges commands / states between ROS and the FRI.
   *
   * Calls step() on #app_, which callbacks #lbr_client_. #lbr_client_ reads commands / write states
   * through real-time safe topics.
   *
   */
  void run_();

  rclcpp::Node::SharedPtr node_; /**< Node handle.*/

  std::unique_ptr<std::thread> run_thread_; /**< Thread running the #run_ method.*/

  const char *remote_host_; /**< The remote host's IP address.*/
  int port_id_;             /**< The UDP port id.*/

  std::atomic<bool> connected_; /**< True if UDP port open and communication running.*/

  rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr
      app_connect_srv_; /**< Service to connect to robot via #on_app_connect_ callback.*/
  rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr
      app_disconnect_srv_; /**< Service to disconnect from robot via #on_app_disconnect_ callback.*/

  std::shared_ptr<LBRClient> lbr_client_; /**< Writes commands to / reads states from robot.*/
  std::unique_ptr<KUKA::FRI::UdpConnection>
      connection_; /**< UDP connection for reading states / writing commands.*/
  std::unique_ptr<KUKA::FRI::ClientApplication>
      app_; /**< FRI client application that callbacks #lbr_client_ methods.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_APP_HPP_
