#ifndef LBR_DEMOS_ADVANCED_CPP__LBR_BASE_POSITION_COMMAND_NODE_HPP_
#define LBR_DEMOS_ADVANCED_CPP__LBR_BASE_POSITION_COMMAND_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_idl/msg/lbr_joint_position_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"

namespace lbr_demos {
class LBRBasePositionCommandNode : public rclcpp::Node {
public:
  LBRBasePositionCommandNode(const std::string &node_name, const rclcpp::NodeOptions &options)
      : Node(node_name, options) {
    // retrieve parameters
    robot_description_ =
        this->retrieve_parameter_("robot_state_publisher", "robot_description").as_string();
    update_rate_ = this->retrieve_parameter_("controller_manager", "update_rate").as_int();
    dt_ = 1.0 / static_cast<double>(update_rate_);

    // publishers and subscribers
    lbr_joint_position_command_pub_ =
        create_publisher<lbr_fri_idl::msg::LBRJointPositionCommand>("command/joint_position", 1);

    lbr_state_sub_ = create_subscription<lbr_fri_idl::msg::LBRState>(
        "state", 1,
        std::bind(&LBRBasePositionCommandNode::on_lbr_state_, this, std::placeholders::_1));
  }

protected:
  rclcpp::Publisher<lbr_fri_idl::msg::LBRJointPositionCommand>::SharedPtr
      lbr_joint_position_command_pub_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRState>::SharedPtr lbr_state_sub_;

  std::string robot_description_;
  int64_t update_rate_;
  double dt_;

protected:
  virtual void on_lbr_state_(const lbr_fri_idl::msg::LBRState::SharedPtr lbr_state) = 0;

  rclcpp::Parameter retrieve_parameter_(const std::string &remote_node_name,
                                        const std::string &parameter_name) {
    rclcpp::AsyncParametersClient parameter_client(this, remote_node_name);
    while (!parameter_client.wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        std::string err = "Interrupted while waiting for the service. Exiting.";
        RCLCPP_ERROR(this->get_logger(), err.c_str());
        throw std::runtime_error(err);
      }
      RCLCPP_INFO(this->get_logger(), "Wating for '%s' service...", remote_node_name.c_str());
    }
    auto future = parameter_client.get_parameters({parameter_name});
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      std::string err = "Failed to retrieve '" + parameter_name + "'.";
      RCLCPP_ERROR(this->get_logger(), parameter_name.c_str());
      throw std::runtime_error(err);
    }
    RCLCPP_INFO(this->get_logger(), "Received '%s' from '%s'.", parameter_name.c_str(),
                remote_node_name.c_str());
    return future.get()[0];
  }
};
} // end of namespace lbr_demos
#endif // LBR_DEMOS_ADVANCED_CPP__LBR_BASE_POSITION_COMMAND_NODE_HPP_
