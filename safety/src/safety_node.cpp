// Standard lib
#include <chrono>
#include <functional>
#include <string>
#include <iostream>

// ROS
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

// Robot model
#include "safety/robot.hpp"

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node {

public:

  SafetyNode() : Node("safety_node") {

    // Load URDF from parameter
    this->declare_parameter("robot_description");
    std::string urdf = this->get_parameter("robot_description").as_string();

    // Setup robot model
    _robot = std::make_unique<RobotModel>(urdf);

    // Setup ROS subscriber
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&SafetyNode::joint_state_callback, this, _1));
    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/command", 10, std::bind(&SafetyNode::command_callback, this, _1));

  }

private:

  std::unique_ptr<RobotModel> _robot;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    double t = this->now().seconds(); // time message recieved

  }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
