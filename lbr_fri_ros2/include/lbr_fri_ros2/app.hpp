#ifndef LBR_FRI_ROS2__APP_HPP_
#define LBR_FRI_ROS2__APP_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

#include "lbr_fri_ros2/factories.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief The App has a node for exposing FRI methods to services. It shares this node with the
 * #client_, which reads commands / write states via realtime safe topics.
 *
 * Services:
 * - <b>connect</b> (lbr_fri_msgs::srv::AppConnect)
 * Opens UDP port to FRI. Creates #run_thread_ thread via #on_app_connect_ that calls #run_ to
 * communicate with the robot.
 * - <b>disconnect</b> (lbr_fri_msgs::srv::AppDisconnect)
 * Closes UDP port to FRI. Finishes #run_thread_ thread via #on_app_disconnect_ through ending
 * #run_.
 *
 */
class App {
public:
  /**
   * @brief Construct a new App object.
   *
   * @param node Shared node
   *
   * @throws std::runtime error if no robot_description in node parameters
   */
  App(const rclcpp::Node::SharedPtr node);

  App(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr);

  ~App();

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
  bool connect(const int &port_id = 30200, const char *const remote_host = NULL);

  /**
   * @brief Closes the UDP port and joins the #run_thread_.
   *
   * @return true if closed successfully / already closed
   * @return false if failed to close
   *
   * @throws std::runtime_error if #run_thread_ fails to join
   *
   */
  bool disconnect();

  /**
   * @brief Exchanges commands / states between ROS and the FRI.
   *
   * Calls step() on #app_, which callbacks #client_. #client_ reads commands / write
   * states through realtime safe topics.
   *
   */
  void run(int rt_prio);

protected:
  /**
   * @brief Checks for valid port id.
   *
   * @param[in] port_id The port id
   * @return true if port id in [30200, 30209]
   * @return false else
   */
  bool valid_port_(const int &port_id);

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  std::unique_ptr<std::thread> run_thread_; /**< Thread running the #run_ method.*/

  std::atomic<bool> connected_; /**< True if UDP port open and communication running.*/

  std::shared_ptr<KUKA::FRI::LBRClient>
      client_; /**< Writes commands to / reads states from robot.*/
  std::unique_ptr<KUKA::FRI::UdpConnection>
      connection_; /**< UDP connection for reading states / writing commands.*/
  std::unique_ptr<KUKA::FRI::ClientApplication>
      app_; /**< FRI client application that callbacks #client_ methods.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__APP_HPP_
