#include "lbr_ros2_control/lbr_virtual_ft_broadcaster.hpp"

namespace lbr_ros2_control {
controller_interface::InterfaceConfiguration
LBRVirtualFTBroadcaster::command_interface_configuration() const override;

controller_interface::InterfaceConfiguration
LBRVirtualFTBroadcaster::state_interface_configuration() const override;

controller_interface::CallbackReturn LBRVirtualFTBroadcaster::on_init() override;

controller_interface::return_type
LBRVirtualFTBroadcaster::update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

controller_interface::CallbackReturn
LBRVirtualFTBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state) override;

controller_interface::CallbackReturn
LBRVirtualFTBroadcaster::on_activate(const rclcpp_lifecycle::State &previous_state) override;

controller_interface::CallbackReturn
LBRVirtualFTBroadcaster::on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
} // end of namespace lbr_ros2_control