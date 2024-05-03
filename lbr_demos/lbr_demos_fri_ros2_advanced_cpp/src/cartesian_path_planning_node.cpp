#include <cmath>
#include <iostream>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class CartesianPosePublisherNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_subscriber_;

  geometry_msgs::msg::Pose initial_cartesian_pose_; // robot starts from this pose
  bool is_init_;

  double amplitude_;     // rad
  double frequency_;     // Hz
  double sampling_time_; // sampling time for sending position command
  double phase_;         // initial phase

private:
  /**
   * @function: callback function for Cartesian Pose Subscriber
   * @param msg Cartesian Pose of the robot
   */
  void on_cartesian_pose(const geometry_msgs::msg::Pose &msg) {
    if (!is_init_) {
      initial_cartesian_pose_ = msg;
      is_init_ = true;
    } else {
      geometry_msgs::msg::Pose cartesian_pose_command = initial_cartesian_pose_;

      phase_ = phase_ + 2 * M_PI * frequency_ * sampling_time_;
      cartesian_pose_command.position.z += amplitude_ * sin(phase_);

      cartesian_pose_publisher_->publish(cartesian_pose_command);
    }

    return;
  }

public:
  CartesianPosePublisherNode() : Node("cartesian_pose_publisher_node") {
    is_init_ = false;
    amplitude_ = 0.05;
    frequency_ = 0.5;
    sampling_time_ = 0.01;
    phase_ = 0.0;

    cartesian_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::Pose>("/lbr/command/cartesian_pose", 1);
    cartesian_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/lbr/state/cartesian_pose", 1,
        std::bind(&CartesianPosePublisherNode::on_cartesian_pose, this, _1));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CartesianPosePublisherNode>());

  rclcpp::shutdown();
  return 0;
}
