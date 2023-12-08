#ifndef LBR_FRI_ROS2__APP_COMPONENT_HPP_
#define LBR_FRI_ROS2__APP_COMPONENT_HPP_

#include <algorithm>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/msg/lbr_torque_command.hpp"
#include "lbr_fri_msgs/msg/lbr_wrench_command.hpp"
#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/enum_maps.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/state_interface.hpp"

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
  void connect_(const int &port_id = 30200, const char *const remote_host = NULL,
                const uint8_t &rt_prio = 80, const uint8_t &max_attempts = 10);

  // command buffer
  lbr_fri_msgs::msg::LBRCommand lbr_command_; /**< The command to be sent to the robot.*/

  // command subscriptions
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
  bool on_command_checks_(const int &expected_command_mode);

  // state publisher
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr
      state_pub_;                                /**< Publishes the robot state.*/
  rclcpp::TimerBase::SharedPtr state_pub_timer_; /**< Timer for publishing the robot state.*/
  void on_state_pub_timer_();                    /**< Callback for publishing the robot state.*/

  // services
  rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr
      app_connect_srv_; /**< Service to connect to robot via #on_app_connect_ callback.*/
  rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr
      app_disconnect_srv_; /**< Service to disconnect from robot via #on_app_disconnect_
                              callback.*/

  void on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                       lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

  void on_app_disconnect_(const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr request,
                          lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

  // app
  rclcpp::Node::SharedPtr app_node_ptr_; /** Node for communicating with ROS.<*/
  std::shared_ptr<AsyncClient> async_client_ptr_;
  std::unique_ptr<App> app_ptr_; /** #lbr_fri_ros2::App for communicating with the hardware.<*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMPONENT_HPP_
