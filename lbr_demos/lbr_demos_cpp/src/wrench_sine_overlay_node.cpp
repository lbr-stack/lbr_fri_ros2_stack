#include <functional>
#include <math.h>
#include <memory>
#include <string>

// include fri for session state
#include "friClientIf.h"

#include "rclcpp/rclcpp.hpp"

// include lbr_fri_msgs
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/msg/lbr_wrench_command.hpp"

class WrenchSineOverlayNode : public rclcpp::Node {
  static constexpr double amplitude_x_ = 5.;   // N
  static constexpr double amplitude_y_ = 5.;   // N
  static constexpr double frequency_x_ = 0.25; // Hz
  static constexpr double frequency_y_ = 0.25; // Hz

public:
  WrenchSineOverlayNode(const std::string &node_name)
      : Node(node_name), phase_x_(0.), phase_y_(0.) {
    // create publisher to /lbr/command/wrench
    lbr_wrench_command_pub_ =
        this->create_publisher<lbr_fri_msgs::msg::LBRWrenchCommand>("/lbr/command/wrench", 1);

    // create subscription to /lbr/state
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state", 1,
        std::bind(&WrenchSineOverlayNode::on_lbr_state_, this, std::placeholders::_1));
  };

protected:
  void on_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    lbr_wrench_command_.joint_position = lbr_state->ipo_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay wrench sine wave on x / y direction
      lbr_wrench_command_.wrench[0] = amplitude_x_ * sin(phase_x_);
      lbr_wrench_command_.wrench[1] = amplitude_y_ * sin(phase_y_);
      phase_x_ += 2 * M_PI * frequency_x_ * lbr_state->sample_time;
      phase_y_ += 2 * M_PI * frequency_y_ * lbr_state->sample_time;

      lbr_wrench_command_pub_->publish(lbr_wrench_command_);
    } else {
      // reset phase
      phase_x_ = 0.;
      phase_y_ = 0.;
    }
  };

  double phase_x_, phase_y_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRWrenchCommand>::SharedPtr lbr_wrench_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  lbr_fri_msgs::msg::LBRWrenchCommand lbr_wrench_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchSineOverlayNode>("wrench_sine_overlay_node"));
  rclcpp::shutdown();
  return 0;
};
