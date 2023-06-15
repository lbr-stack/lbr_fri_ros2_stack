#include <functional>
#include <math.h>
#include <memory>
#include <string>

// include fri for session state
#include "fri/friClientIf.h"

#include "rclcpp/rclcpp.hpp"

// include lbr_fri_msgs
#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

class JointSineOverlayNode : public rclcpp::Node {
  static constexpr double amplitude_ = 0.04; // rad
  static constexpr double frequency_ = 0.25; // Hz

public:
  JointSineOverlayNode(const std::string &node_name) : Node(node_name), phase_(0.) {
    // create publisher to /lbr_command
    lbr_command_pub_ = this->create_publisher<lbr_fri_msgs::msg::LBRCommand>(
        "/lbr_command", rclcpp::QoS(1)
                            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                            .deadline(std::chrono::milliseconds(10)));

    // create subscription to /lbr_state
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state",
        rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .deadline(std::chrono::milliseconds(10)),
        std::bind(&JointSineOverlayNode::lbr_state_cb_, this, std::placeholders::_1));
  };

protected:
  void lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    lbr_command_.joint_position = lbr_state->ipo_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay sine wave on 4th joint
      lbr_command_.joint_position[3] += amplitude_ * sin(phase_);
      phase_ += 2 * M_PI * frequency_ * lbr_state->sample_time;

      lbr_command_pub_->publish(lbr_command_);
    } else {
      // reset phase
      phase_ = 0.;
    }
  };

  double phase_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  lbr_fri_msgs::msg::LBRCommand lbr_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSineOverlayNode>("joint_sine_overlay_node"));
  rclcpp::shutdown();
  return 0;
};
