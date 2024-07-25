#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Configure node
  auto node_ptr = rclcpp::Node::make_shared("hello_moveit");
  node_ptr->declare_parameter("robot_name", "lbr");
  auto robot_name = node_ptr->get_parameter("robot_name").as_string();

  // Create MoveGroupInterface (lives inside robot_name namespace)
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
      node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

  // Set a target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = -0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.9;
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto error_code = move_group_interface.plan(plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
    // Execute the plan
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(node_ptr->get_logger(), "Failed to plan to target pose");
  }

  rclcpp::shutdown();
  return 0;
}
