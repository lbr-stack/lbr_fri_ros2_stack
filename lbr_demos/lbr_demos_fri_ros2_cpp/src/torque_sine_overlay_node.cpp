#include <functional>
#include <math.h>
#include <memory>
#include <string>

// include fri for session state
#include "friClientIf.h"

#include "rclcpp/rclcpp.hpp"

// include lbr_fri_msgs
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/msg/lbr_torque_command.hpp"

class TorqueSineOverlayNode : public rclcpp::Node {
  static constexpr double amplitude_ = 15.;  // Nm
  static constexpr double frequency_ = 0.25; // Hz

public:
  TorqueSineOverlayNode(const std::string &node_name) : Node(node_name), phase_(0.) {
    // create publisher to /lbr/command/torque
    lbr_torque_command_pub_ =
        this->create_publisher<lbr_fri_msgs::msg::LBRTorqueCommand>("/lbr/command/torque", 1);

    // create subscription to /lbr/state
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state", 1,
        std::bind(&TorqueSineOverlayNode::on_lbr_state_, this, std::placeholders::_1));
  };

protected:
  void on_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    lbr_torque_command_.joint_position = lbr_state->ipo_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay torque sine wave on 4th joint
      lbr_torque_command_.torque[3] = amplitude_ * sin(phase_);
      phase_ += 2 * M_PI * frequency_ * lbr_state->sample_time;

      lbr_torque_command_pub_->publish(lbr_torque_command_);
    } else {
      // reset phase
      phase_ = 0.;
    }
  };

  double phase_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRTorqueCommand>::SharedPtr lbr_torque_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  lbr_fri_msgs::msg::LBRTorqueCommand lbr_torque_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueSineOverlayNode>("torque_sine_overlay_node"));
  rclcpp::shutdown();
  return 0;
};
