#ifndef LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_
#define LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_

#include <limits>
#include <memory>
#include <stdexcept>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "friClientIf.h"

#include "lbr_ros2_control/lbr_system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRVirtualFTBroadcaster : public controller_interface::ControllerInterface {
public:
  LBRVirtualFTBroadcaster() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_publisher_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      rt_wrench_stamped_publisher_ptr_;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_
