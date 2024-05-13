#ifndef LBR_FRI_ROS2__UTILS_HPP_
#define LBR_FRI_ROS2__UTILS_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace lbr_fri_ros2 {
rclcpp::Parameter retrieve_paramter(const rclcpp::Node::SharedPtr node,
                                    const std::string &remote_node_name,
                                    const std::string &parameter_name) {
  rclcpp::AsyncParametersClient parameter_client(node, remote_node_name);
  while (!parameter_client.wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      std::string err = "Interrupted while waiting for the service. Exiting.";
      RCLCPP_ERROR(node->get_logger(), err.c_str());
      throw std::runtime_error(err);
    }
    RCLCPP_INFO(node->get_logger(), "Wating for '%s' service...", remote_node_name.c_str());
  }
  auto future = parameter_client.get_parameters({parameter_name});
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    std::string err = "Failed to retrieve '%s'.";
    RCLCPP_ERROR(node->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
  return future.get()[0];
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__UTILS_HPP_
