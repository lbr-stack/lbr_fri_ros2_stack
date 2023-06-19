#ifndef LBR_FRI_ROS2__LBR_APP_COMPONENT_HPP_
#define LBR_FRI_ROS2__LBR_APP_COMPONENT_HPP_
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_ros2/lbr_app.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Component for instantiating #LBRApp.
 *
 */
class LBRAppComponent {
public:
  /**
   * @brief Construct a new LBRAppComponent object.
   *
   * @param options Node options
   */
  LBRAppComponent(const rclcpp::NodeOptions &options);

  /**
   * @brief Get the node base interface object. Implementing this is necessary for components via
   * composition.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  std::unique_ptr<lbr_fri_ros2::LBRApp>
      lbr_app_;                  /** #LBRApp for communicating with the hardware.<*/
  rclcpp::Node::SharedPtr node_; /** Node for communicating with ROS.<*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMPONENT_HPP_
