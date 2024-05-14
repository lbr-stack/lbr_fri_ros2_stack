#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_idl/msg/lbr_state.hpp"

#include "admittance_controller.hpp"
#include "lbr_base_position_command_node.hpp"

namespace lbr_demos {
class AdmittanceControlNode : public LBRBasePositionCommandNode {
public:
  AdmittanceControlNode(const rclcpp::NodeOptions &options)
      : LBRBasePositionCommandNode("admittance_control", options) {
    this->declare_parameter<std::string>("base_link", "link_0");
    this->declare_parameter<std::string>("end_effector_link", "link_ee");
    this->declare_parameter<std::vector<double>>("f_ext_th", {2., 2., 2., 0.5, 0.5, 0.5});
    this->declare_parameter<std::vector<double>>("dq_gains", {2., 2., 2., 2., 2., 2., 2.});
    this->declare_parameter<std::vector<double>>("dx_gains", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<double>("exp_smooth", 0.95);

    exp_smooth_ = this->get_parameter("exp_smooth").as_double();
    if (exp_smooth_ < 0. || exp_smooth_ > 1.) {
      throw std::runtime_error("Invalid exponential smoothing factor.");
    }

    admittance_controller_ = std::make_unique<AdmittanceController>(
        this->robot_description_, this->get_parameter("base_link").as_string(),
        this->get_parameter("end_effector_link").as_string(),
        this->get_parameter("f_ext_th").as_double_array(),
        this->get_parameter("dq_gains").as_double_array(),
        this->get_parameter("dx_gains").as_double_array());

    // log parameters to terminal
    this->log_paramters_();
  }

protected:
  void log_paramters_() {
    RCLCPP_INFO(this->get_logger(), "*** Parameters:");
    RCLCPP_INFO(this->get_logger(), "*   base_link: %s",
                this->get_parameter("base_link").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "*   end_effector_link: %s",
                this->get_parameter("end_effector_link").as_string().c_str());
    auto f_ext_th = this->get_parameter("f_ext_th").as_double_array();
    RCLCPP_INFO(this->get_logger(), "*   f_ext_th: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f", f_ext_th[0],
                f_ext_th[1], f_ext_th[2], f_ext_th[3], f_ext_th[4], f_ext_th[5]);
    auto dq_gains = this->get_parameter("dq_gains").as_double_array();
    RCLCPP_INFO(this->get_logger(), "*   dq_gains: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",
                dq_gains[0], dq_gains[1], dq_gains[2], dq_gains[3], dq_gains[4], dq_gains[5],
                dq_gains[6]);
    auto dx_gains = this->get_parameter("dx_gains").as_double_array();
    RCLCPP_INFO(this->get_logger(), "*   dx_gains: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f", dx_gains[0],
                dx_gains[1], dx_gains[2], dx_gains[3], dx_gains[4], dx_gains[5]);
  }

  void on_lbr_state_(const lbr_fri_idl::msg::LBRState::SharedPtr lbr_state) override {
    if (!lbr_state) {
      return;
    }

    smooth_lbr_state_(lbr_state);

    auto lbr_command = admittance_controller_->update(lbr_state_, dt_);
    lbr_joint_position_command_pub_->publish(lbr_command);
  };

  void smooth_lbr_state_(const lbr_fri_idl::msg::LBRState::SharedPtr lbr_state) {
    if (!init_) {
      lbr_state_ = *lbr_state;
      init_ = true;
      return;
    }

    for (int i = 0; i < 7; i++) {
      lbr_state_.measured_joint_position[i] =
          lbr_state->measured_joint_position[i] * (1 - exp_smooth_) +
          lbr_state_.measured_joint_position[i] * exp_smooth_;
      lbr_state_.external_torque[i] = lbr_state->external_torque[i] * (1 - exp_smooth_) +
                                      lbr_state_.external_torque[i] * exp_smooth_;
    }
  }

  double exp_smooth_;
  bool init_{false};
  lbr_fri_idl::msg::LBRState lbr_state_;

  std::unique_ptr<AdmittanceController> admittance_controller_;
};
} // end of namespace lbr_demos

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lbr_demos::AdmittanceControlNode)
