#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/app.hpp"

#include "admittance_controller.hpp"

namespace lbr_fri_ros2 {
class AdmittanceControlNode : public rclcpp::Node {
public:
  AdmittanceControlNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("admittance_control_node", options) {
    this->declare_parameter<std::string>("robot_description");
    this->declare_parameter<std::string>("base_link", "link_0");
    this->declare_parameter<std::string>("end_effector_link", "link_ee");
    this->declare_parameter<std::vector<double>>("f_ext_th", {2., 2., 2., 0.5, 0.5, 0.5});
    this->declare_parameter<std::vector<double>>("dq_gains", {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8});
    this->declare_parameter<std::vector<double>>("dx_gains", {4.0, 4.0, 4.0, 40., 40., 40.});

    admittance_controller_ =
        std::make_unique<AdmittanceController>(this->get_parameter("robot_description").as_string(),
                                               this->get_parameter("base_link").as_string(),
                                               this->get_parameter("end_effector_link").as_string(),
                                               this->get_parameter("f_ext_th").as_double_array(),
                                               this->get_parameter("dq_gains").as_double_array(),
                                               this->get_parameter("dx_gains").as_double_array());

    lbr_position_command_pub_ =
        create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>("/lbr/command/joint_position", 1);
    lbr_state_sub_ = create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state", 1,
        std::bind(&AdmittanceControlNode::on_lbr_state, this, std::placeholders::_1));
  }

protected:
  void on_lbr_state(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    if (!lbr_state) {
      return;
    }

    smooth_lbr_state_(lbr_state, 0.95);

    auto lbr_command = admittance_controller_->update(lbr_state_);
    lbr_position_command_pub_->publish(lbr_command);
  };

  void smooth_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state, double alpha) {
    if (!init_) {
      lbr_state_ = *lbr_state;
      init_ = true;
      return;
    }

    for (int i = 0; i < 7; i++) {
      lbr_state_.measured_joint_position[i] = lbr_state->measured_joint_position[i] * (1 - alpha) +
                                              lbr_state_.measured_joint_position[i] * alpha;
      lbr_state_.external_torque[i] =
          lbr_state->external_torque[i] * (1 - alpha) + lbr_state_.external_torque[i] * alpha;
    }
  }

  bool init_{false};
  lbr_fri_msgs::msg::LBRState lbr_state_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr lbr_position_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  std::unique_ptr<AdmittanceController> admittance_controller_;
};
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::AdmittanceControlNode)
