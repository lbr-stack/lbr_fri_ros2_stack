#ifndef LBR_ROS2_CONTROL__TWIST_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__TWIST_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "friLBRState.h"

#include "lbr_fri_ros2/control.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/math.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class TwistController : public controller_interface::ControllerInterface {
public:
  TwistController();

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
  bool assign_state_interfaces_();
  void release_state_interfaces_();
  void reset_command_buffer_();
  void zero_joint_velocity_command_();
  void configure_joint_names_();
  void configure_joint_limits_();
  void configure_inv_jac_ctrl_impl_();
  void log_info_() const;

  // some safety checks
  std::atomic<int> updates_since_last_command_ = 0;
  double timeout_ = 0.2;

  // joint veloctiy computation
  std::unique_ptr<lbr_fri_ros2::InvJacCtrlImpl> inv_jac_ctrl_impl_ptr_;
  lbr_fri_ros2::jnt_array_t q_, q_target_, dq_;

  // interfaces
  lbr_fri_ros2::jnt_name_array_t joint_names_;
  lbr_fri_ros2::jnt_array_t lower_joint_limits_, upper_joint_limits_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_;
  std::unique_ptr<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      session_state_interface_ptr_;

  // real-time twist command topic
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> rt_twist_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__TWIST_CONTROLLER_HPP_
