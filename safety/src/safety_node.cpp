#include <chrono>
#include <functional>
#include <string>
#include <iostream>


// #include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>

class SafetyNode : public rclcpp::Node {

public:
  
  SafetyNode() : Node("safety_node") {

    // Load URDF from parameter
    this->declare_parameter("robot_description");
    std::string urdf = this->get_parameter("robot_description").as_string();

    std::cout << urdf << "\n";

  }
  

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}


/*

// Standard lib
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Robot model
#include "safety/robot.hpp"

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node {

public:

  SafetyNode () : Node("safety_node") {

    // Load robot model
    std::string urdf = this->get_parameter("robot_description").get_parameter_value().get<std::string>();
    robot_ = Robot(urdf);

    // Setup ROS publisher/subscriber
    exec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_states/execute", 10);
    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/command", 10, std::bind(&SafetyNode::command_callback, this, _1));

  }

private:

  Robot robot_; // model of robot, see include/safety/robot.hpp
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr exec_pub_; // publishes joint state command directly to the robot, these messages are considered "safe"!
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_; // listens to joint state commands on the topic joint_states/command, uses command_callback as the callback method

  // Joint state command callback
  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {

    // Check if goal is safe
    if (robot_.is_safe(msg)) {

      // command is safe, send goal to robot, and update model
      execute_on_robot(); // point of no return!
      robot_.command_executed();

    }
    else {

      // command is NOT safe, error thrown and goal joint states are not executed on robot
      RCLCPP_ERROR(this->get_logger(), "commanded joint state is not safe [%d]\n%s", robot_.get_status(), robot_.get_reason());

    }

  }

  // Send target command to robot, the target is considered "safe"! >>No additional checks are done here.<<
  void execute_on_robot(){

    // Initialize and populate command message
    auto cmd = std_msgs::msg::Float64MultiArray();
    cmd.data.resize(NDOF);
    for (int i = 0; i < NDOF; ++i) {
      cmd.data.push_back(robot_.get_safe_cmd(i));
    }

    // Execute command
    exec_pub_->publish(cmd);

  }

};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}

*/
