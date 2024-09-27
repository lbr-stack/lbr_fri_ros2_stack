#include <functional>
#include <math.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// include fri for session state
#include "friClientIf.h"

// include lbr_fri_idl
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_idl/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/utils.hpp"

class WrenchSineOverlay {
  static constexpr double amplitude_x_ = 5.;   // N
  static constexpr double amplitude_y_ = 5.;   // N
  static constexpr double frequency_x_ = 0.25; // Hz
  static constexpr double frequency_y_ = 0.25; // Hz

public:
  WrenchSineOverlay(const rclcpp::Node::SharedPtr node) : node_(node), phase_x_(0.), phase_y_(0.) {
    // create publisher to command/wrench
    lbr_wrench_command_pub_ =
        node_->create_publisher<lbr_fri_idl::msg::LBRWrenchCommand>("command/wrench", 1);

    // create subscription to state
    lbr_state_sub_ = node_->create_subscription<lbr_fri_idl::msg::LBRState>(
        "state", 1, std::bind(&WrenchSineOverlay::on_lbr_state_, this, std::placeholders::_1));

    // get control rate from controller_manager
    auto update_rate =
        lbr_fri_ros2::retrieve_paramter(node_, "controller_manager", "update_rate").as_int();
    dt_ = 1.0 / static_cast<double>(update_rate);
  };

protected:
  void on_lbr_state_(const lbr_fri_idl::msg::LBRState::SharedPtr lbr_state) {
    if (!lbr_state_init_) {
      lbr_state_ = *lbr_state;
      lbr_state_init_ = true;
    }
    lbr_wrench_command_.joint_position = lbr_state_.measured_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay wrench sine wave on x / y direction
      lbr_wrench_command_.wrench[0] = amplitude_x_ * sin(phase_x_);
      lbr_wrench_command_.wrench[1] = amplitude_y_ * sin(phase_y_);
      phase_x_ += 2 * M_PI * frequency_x_ * dt_;
      phase_y_ += 2 * M_PI * frequency_y_ * dt_;

      lbr_wrench_command_pub_->publish(lbr_wrench_command_);
    } else {
      // reset phase
      phase_x_ = 0.;
      phase_y_ = 0.;
    }
  };

protected:
  rclcpp::Node::SharedPtr node_;
  double dt_;
  double phase_x_, phase_y_;
  rclcpp::Publisher<lbr_fri_idl::msg::LBRWrenchCommand>::SharedPtr lbr_wrench_command_pub_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRState>::SharedPtr lbr_state_sub_;
  bool lbr_state_init_ = false;
  lbr_fri_idl::msg::LBRState lbr_state_;
  lbr_fri_idl::msg::LBRWrenchCommand lbr_wrench_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wrench_sine_overlay");
  WrenchSineOverlay wrench_sine_overlay(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};
