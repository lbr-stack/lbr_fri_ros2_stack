#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lbr_demos {
class PosePlanningNode : public rclcpp::Node {
protected:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

  geometry_msgs::msg::Pose initial_pose_; // robot starts from this pose
  bool is_init_;

  double amplitude_;     // rad
  double frequency_;     // Hz
  double sampling_time_; // sampling time for sending position command
  double phase_;         // initial phase

public:
  PosePlanningNode(const rclcpp::NodeOptions &options) : Node("pose_planning", options) {
    is_init_ = false;
    amplitude_ = 0.05;
    frequency_ = 0.5;
    sampling_time_ = 0.01;
    phase_ = 0.0;

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("command/pose", 1);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "state/pose", 1, std::bind(&PosePlanningNode::on_pose_, this, std::placeholders::_1));
  }

protected:
  void on_pose_(const geometry_msgs::msg::Pose &msg) {
    if (!is_init_) {
      initial_pose_ = msg;
      is_init_ = true;
    } else {
      geometry_msgs::msg::Pose cartesian_pose_command = initial_pose_;

      phase_ = phase_ + 2 * M_PI * frequency_ * sampling_time_;
      cartesian_pose_command.position.z += amplitude_ * sin(phase_);

      pose_pub_->publish(cartesian_pose_command);
    }
  }
};
} // namespace lbr_demos

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lbr_demos::PosePlanningNode)
