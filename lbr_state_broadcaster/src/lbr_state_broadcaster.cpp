#include <lbr_state_broadcaster/lbr_state_broadcaster.hpp>


namespace lbr_state_broadcaster {

controller_interface::return_type
LBRStateBroadcaster::init(const std::string& controller_name) {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LBRStateBroadcaster::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE
    };
}

controller_interface::InterfaceConfiguration
LBRStateBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interface_configuration;
    return state_interface_configuration;
}

controller_interface::return_type 
LBRStateBroadcaster::update() {
    
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state
) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state
) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& previous_state
) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

PLUGINLIB_EXPORT_CLASS(lbr_state_broadcaster::LBRStateBroadcaster, controller_interface::ControllerInterface)

} // end of namespace lbr_state_broadcaster
