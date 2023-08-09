#ifndef LBR_FRI_ROS2__APP_COMPONENT_HPP_
#define LBR_FRI_ROS2__APP_COMPONENT_HPP_
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Component for instantiating #lbr_fri_ros2::App.
 *
 */
class AppComponent {
public:
  /**
   * @brief Construct a new AppComponent object.
   *
   * @param options Node options
   */
  AppComponent(const rclcpp::NodeOptions &options);

  /**
   * @brief Get the node base interface object. Implementing this is necessary for components via
   * composition.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  std::unique_ptr<lbr_fri_ros2::App>
      lbr_app_;                      /** #lbr_fri_ros2::App for communicating with the hardware.<*/
  rclcpp::Node::SharedPtr lbr_node_; /** Node for communicating with ROS.<*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMPONENT_HPP_
