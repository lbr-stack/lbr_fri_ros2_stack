#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <lbr_state_msgs/msg/lbr_state.hpp>


namespace lbr_state_broadcaster {

class LBRStateBroadcaster : public controller_interface::ControllerInterface {

    public:
        LBRStateBroadcaster() = default;

        controller_interface::return_type init(const std::string& controller_name) override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        
        controller_interface::return_type update() override;
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state
        ) override;
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state
        ) override;
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state
        ) override;

    protected:
        std::vector<std::string> joint_names_;
        std::shared_ptr<rclcpp::Publisher<lbr_state_msgs::msg::LBRState>> lbr_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<lbr_state_msgs::msg::LBRState>> realtime_lbr_state_publisher_;
};

} // end of namespace lbr_state_broadcaster
