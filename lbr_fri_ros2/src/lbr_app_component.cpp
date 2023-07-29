#include "lbr_app_component.hpp"

namespace lbr_fri_ros2 {
LBRAppComponent::LBRAppComponent(const rclcpp::NodeOptions &options) {
  lbr_node_ = std::make_shared<rclcpp::Node>("lbr", options);
  lbr_app_ = std::make_unique<lbr_fri_ros2::LBRApp>(lbr_node_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
LBRAppComponent::get_node_base_interface() const {
  return lbr_node_->get_node_base_interface();
}
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::LBRAppComponent)
