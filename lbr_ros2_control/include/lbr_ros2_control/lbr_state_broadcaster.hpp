#ifndef LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_
#define LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_

#include <limits>
#include <memory>
#include <stdexcept>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_ros2_control/lbr_state_interface_wrapper.hpp"

namespace lbr_ros2_control {
class LBRStateBroadcaster : public controller_interface::ControllerInterface {
public:
  LBRStateBroadcaster() = default;

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
  LBRStateInterfaceWrapper state_interface_wrapper_;

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr state_publisher_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>
      rt_state_publisher_ptr_;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_STATE_BROADCASTER_HPP_
