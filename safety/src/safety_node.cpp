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

const double NSECS_TO_SECS=1000000000.0;

class SafetyNode : public rclcpp::Node {

public:
  
  SafetyNode() : Node("safety_node") {

    t_prev = 0;

    // Load URDF from parameter
    this->declare_parameter("robot_description");
    std::string urdf = this->get_parameter("robot_description").as_string();

    // Setup robot model
    _robot = std::make_unique<RobotModel>(urdf);

    // Setup ROS subscriber
    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/command", 10, std::bind(&SafetyNode::command_callback, this, _1));

  }  

private:

  unsigned long long int t_prev;

  std::unique_ptr<RobotModel> _robot;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    std::cout << "i am here!\n";
    unsigned long long int t = msg->header.stamp.nanosec;

    unsigned long long int dti = t - t_prev;

    long double dt = static_cast<long double>(dti)/NSECS_TO_SECS;

    
    
   
    std::cout << "time=" << dt << "\n";
    for (std::string name : msg->name) {
      std::cout << "name =" << name << "\n";
    }

    t_prev = t;
    
  }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
