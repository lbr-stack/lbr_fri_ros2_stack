#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

class LBRCommandNode : public rclcpp::Node {
public:
  LBRCommandNode(const std::string &node_name = "lbr_command_node")
      : rclcpp::Node(node_name), initial_lbr_state_(nullptr) {
    declare_parameter("amplitude", M_PI_4);
    declare_parameter("period", 20.);

    amplitude_ = get_parameter("amplitude").as_double();
    omega_ = 2. * M_PI / get_parameter("period").as_double();

    lbr_state_sub_ = create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SensorDataQoS(),
        std::bind(&LBRCommandNode::lbr_state_sub_cb_, this, std::placeholders::_1));

    lbr_command_pub_ =
        create_publisher<lbr_fri_msgs::msg::LBRCommand>("/lbr_command", rclcpp::SensorDataQoS());

    lbr_command_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&LBRCommandNode::timer_cb_, this));
  };

protected:
  void lbr_state_sub_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    if (!initial_lbr_state_) {
      t0_ = static_cast<double>(now().nanoseconds());
      initial_lbr_state_ = lbr_state;
    }
  }

  void timer_cb_() {
    if (!initial_lbr_state_) {
      return;
    }
    lbr_fri_msgs::msg::LBRCommand lbr_command;
    lbr_command.joint_position = initial_lbr_state_->measured_joint_position;
    double t = static_cast<double>(now().nanoseconds() - t0_) / 1.e9;
    lbr_command.joint_position[6] += amplitude_ * std::sin(omega_ * t);
    lbr_command_pub_->publish(lbr_command);
  }

  double amplitude_;
  double omega_;
  double t0_;

  lbr_fri_msgs::msg::LBRState::SharedPtr initial_lbr_state_;

  rclcpp::TimerBase::SharedPtr lbr_command_timer_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LBRCommandNode>());
  rclcpp::shutdown();
  return 0;
}
