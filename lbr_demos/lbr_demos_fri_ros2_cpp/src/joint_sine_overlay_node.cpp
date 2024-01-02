#include <functional>
#include <math.h>
#include <memory>
#include <string>

// include fri for session state
#include "friClientIf.h"

#include "rclcpp/rclcpp.hpp"

// include lbr_fri_msgs
#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

class JointSineOverlayNode : public rclcpp::Node {
  static constexpr double amplitude_ = 0.04; // rad
  static constexpr double frequency_ = 0.25; // Hz

public:
  JointSineOverlayNode(const std::string &node_name) : Node(node_name), phase_(0.) {
    // create publisher to /lbr/command/joint_position
    lbr_position_command_pub_ = this->create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>(
        "/lbr/command/joint_position", 1);

    // create subscription to /lbr/state
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state", 1,
        std::bind(&JointSineOverlayNode::on_lbr_state_, this, std::placeholders::_1));
  };

protected:
  void on_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    lbr_position_command_.joint_position = lbr_state->ipo_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay sine wave on 4th joint
      lbr_position_command_.joint_position[3] += amplitude_ * sin(phase_);
      phase_ += 2 * M_PI * frequency_ * lbr_state->sample_time;

      lbr_position_command_pub_->publish(lbr_position_command_);
    } else {
      // reset phase
      phase_ = 0.;
    }
  };

  double phase_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr lbr_position_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  lbr_fri_msgs::msg::LBRPositionCommand lbr_position_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSineOverlayNode>("joint_sine_overlay_node"));
  rclcpp::shutdown();
  return 0;
};
