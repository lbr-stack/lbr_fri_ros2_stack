#ifndef LBR_FRI_ROS2__APP_COMPONENT_HPP_
#define LBR_FRI_ROS2__APP_COMPONENT_HPP_

#include <algorithm>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/msg/lbr_torque_command.hpp"
#include "lbr_fri_msgs/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Component for instantiating #lbr_fri_ros2::App.
 *
 */
class AppComponent {
public:
  /**
   * @brief Construct a new AppComponent object.
   *
   * @param options Node options
   */
  AppComponent(const rclcpp::NodeOptions &options);

  /**
   * @brief Get the node base interface object. Implementing this is necessary for components via
   * composition.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  // command
  lbr_fri_msgs::msg::LBRCommand lbr_command_; /**< The command to be sent to the robot.*/

  // command subscriber
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr
      position_command_sub_; /**< Subscribes to position commands.*/
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRTorqueCommand>::SharedPtr
      torque_command_sub_; /**< Subscribes to torque commands.*/
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRWrenchCommand>::SharedPtr
      wrench_command_sub_; /**< Subscribes to wrench commands.*/

  void
  on_position_command_(const lbr_fri_msgs::msg::LBRPositionCommand::SharedPtr lbr_position_command);
  void on_torque_command_(const lbr_fri_msgs::msg::LBRTorqueCommand::SharedPtr lbr_torque_command);
  void on_wrench_command_(const lbr_fri_msgs::msg::LBRWrenchCommand::SharedPtr lbr_wrench_command);

  // state publisher
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr
      state_pub_;                                /**< Publishes the robot state.*/
  rclcpp::TimerBase::SharedPtr state_pub_timer_; /**< Timer for publishing the robot state.*/
  void on_state_pub_timer_();                    /**< Callback for publishing the robot state.*/

  // services

  // app
  rclcpp::Node::SharedPtr app_node_; /** Node for communicating with ROS.<*/
  std::shared_ptr<Client> client_ptr_;
  std::unique_ptr<App> app_ptr_; /** #lbr_fri_ros2::App for communicating with the hardware.<*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMPONENT_HPP_

// #ifndef LBR_FRI_ROS2__APP_HPP_
// #define LBR_FRI_ROS2__APP_HPP_

// #include <atomic>
// #include <future>
// #include <memory>
// #include <stdexcept>
// #include <string>
// #include <thread>

// #include "rclcpp/rclcpp.hpp"
// #include "realtime_tools/thread_priority.hpp"

// #include "friClientApplication.h"
// #include "friLBRClient.h"
// #include "friUdpConnection.h"

// #include "lbr_fri_msgs/msg/lbr_command.hpp"
// #include "lbr_fri_msgs/srv/app_connect.hpp"
// #include "lbr_fri_msgs/srv/app_disconnect.hpp"
// #include "lbr_fri_ros2/clients.hpp"
// // #include "lbr_fri_ros2/command_guards.hpp"

// namespace lbr_fri_ros2 {
// /**
//  * @brief The App has a node for exposing FRI methods to services. It shares this node with the
//  * #client_ptr_, which reads commands / write states via realtime safe topics.
//  *
//  * Services:
//  * - <b>connect</b> (lbr_fri_msgs::srv::AppConnect)
//  * Opens UDP port to FRI. Creates #run_thread_ptr_ thread via #on_app_connect_ that calls #run_
//  to
//  * communicate with the robot.
//  * - <b>disconnect</b> (lbr_fri_msgs::srv::AppDisconnect)
//  * Closes UDP port to FRI. Finishes #run_thread_ptr_ thread via #on_app_disconnect_ through
//  ending
//  * #run_.
//  *
//  */
// class App {
// public:
//   /**
//    * @brief Construct a new App object.
//    *
//    * @param node Shared node
//    *
//    * @throws std::runtime error if no robot_description in node parameters
//    */
//   App(const rclcpp::Node::SharedPtr node);
//   ~App();

// protected:
//   /**
//    * @brief Declares parameters for node.
//    *
//    */
//   void declare_parameters_();

//   /**
//    * @brief Gets parameters for node and writes them into members. Checks validity.
//    *
//    * @throws std::range_error for invalid port_id in node parameters
//    */
//   void get_parameters_();

//   /**
//    * @brief Callback to <b>connect</b> service. Calls #connect_.
//    *
//    * @param[in] request Request containing port_id and remote_host
//    * @param[out] response Response containing connected and message
//    */
//   void on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
//                        lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

//   /**
//    * @brief Callback to <b>disconnect</b> service. Calls #disconnect_.
//    *
//    * @param[in] request Empty request
//    * @param[out] response Response containing disconnected and message
//    */
//   void on_app_disconnect_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
//                           lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

//   /**
//    * @brief Checks for valid port id.
//    *
//    * @param[in] port_id The port id
//    * @return true if port id in [30200, 30209]
//    * @return false else
//    */
//   bool valid_port_(const int &port_id);

//   /**
//    * @brief Opens a UDP port and spawns the #run_thread_ptr_.
//    *
//    * @param[in] port_id The port id, allowed values [30200, 30209]
//    * @param[in] remote_host The address of the remote host
//    * @return true if successfully opened / already open
//    * @return false if failed to open
//    *
//    * @throws std::range_error if invalid port_id
//    *
//    */
//   bool connect_(const int &port_id = 30200, const char *const remote_host = NULL);

//   /**
//    * @brief Closes the UDP port and joins the #run_thread_ptr_.
//    *
//    * @return true if closed successfully / already closed
//    * @return false if failed to close
//    *
//    * @throws std::runtime_error if #run_thread_ptr_ fails to join
//    *
//    */
//   bool disconnect_();

//   /**
//    * @brief Exchanges commands / states between ROS and the FRI.
//    *
//    * Calls step() on #app_ptr_, which callbacks #client_ptr_. #client_ptr_ reads commands / write
//    * states through realtime safe topics.
//    *
//    */
//   void run_();

//   rclcpp::Node::SharedPtr node_; /**< Node handle.*/

//   std::unique_ptr<std::thread> run_thread_ptr_; /**< Thread running the #run_ method.*/

//   const char *remote_host_; /**< The remote host's IP address.*/
//   int port_id_;             /**< The UDP port id.*/
//   int rt_prio_;             /**< The realtime priority, read from node parameters.*/

//   std::atomic<bool> connected_; /**< True if UDP port open and communication running.*/

//   rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr
//       app_connect_srv_; /**< Service to connect to robot via #on_app_connect_ callback.*/
//   rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr
//       app_disconnect_srv_; /**< Service to disconnect from robot via #on_app_disconnect_
//       callback.*/

//   std::shared_ptr<Client<lbr_fri_msgs::msg::LBRCommand>>
//       client_ptr_; /**< Writes commands to / reads states from robot.*/
//   std::unique_ptr<KUKA::FRI::UdpConnection>
//       connection_ptr_; /**< UDP connection for reading states / writing commands.*/
//   std::unique_ptr<KUKA::FRI::ClientApplication>
//       app_ptr_; /**< FRI client application that callbacks #client_ptr_ methods.*/
// };
// } // end of namespace lbr_fri_ros2
// #endif // LBR_FRI_ROS2__APP_HPP_
