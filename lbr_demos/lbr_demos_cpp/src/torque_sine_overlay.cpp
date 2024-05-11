#include <functional>
#include <math.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// include fri for session state
#include "friClientIf.h"

// include lbr_fri_idl
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_idl/msg/lbr_torque_command.hpp"
#include "lbr_fri_ros2/utils.hpp"

class TorqueSineOverlay {
  static constexpr double amplitude_ = 15.;  // Nm
  static constexpr double frequency_ = 0.25; // Hz

public:
  TorqueSineOverlay(const rclcpp::Node::SharedPtr node) : node_(node), phase_(0.) {
    // create publisher to command/torque
    lbr_torque_command_pub_ =
        node_->create_publisher<lbr_fri_idl::msg::LBRTorqueCommand>("command/torque", 1);

    // create subscription to state
    lbr_state_sub_ = node_->create_subscription<lbr_fri_idl::msg::LBRState>(
        "state", 1, std::bind(&TorqueSineOverlay::on_lbr_state_, this, std::placeholders::_1));

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
    lbr_torque_command_.joint_position = lbr_state_.measured_joint_position;

    if (lbr_state->session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      // overlay torque sine wave on 4th joint
      lbr_torque_command_.torque[3] = amplitude_ * sin(phase_);
      phase_ += 2 * M_PI * frequency_ * dt_;

      lbr_torque_command_pub_->publish(lbr_torque_command_);
    } else {
      // reset phase
      phase_ = 0.;
    }
  };

protected:
  rclcpp::Node::SharedPtr node_;
  double dt_;
  double phase_;
  rclcpp::Publisher<lbr_fri_idl::msg::LBRTorqueCommand>::SharedPtr lbr_torque_command_pub_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRState>::SharedPtr lbr_state_sub_;
  bool lbr_state_init_ = false;
  lbr_fri_idl::msg::LBRState lbr_state_;
  lbr_fri_idl::msg::LBRTorqueCommand lbr_torque_command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("torque_sine_overlay");
  TorqueSineOverlay torque_sine_overlay(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};
