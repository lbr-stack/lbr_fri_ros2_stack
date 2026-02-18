#ifndef LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_

#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/math.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRWrenchCommandController : public controller_interface::ChainableControllerInterface {
  struct LBRWrenchCommandControllerParameters {
    double max_force_command_norm{10.0};
    double max_torque_command_norm{10.0};
  };

public:
  LBRWrenchCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  // expect full lbr_wrench command in this mode
  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

  // expect just wrench command in this mode
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool assign_state_interfaces_();
  bool assign_command_interfaces_();
  void release_state_interfaces_();
  void release_command_interfaces_();
  void configure_joint_names_();
  void configure_parameters_();
  bool zero_wrench_commands_();
  void init_lbr_wrench_command_subscription_();
  void init_wrench_command_subscription_();
  void reset_lbr_wrench_command_subscription_();
  void reset_wrench_command_subscription_();

  // command limit verifcation
  bool command_in_wrench_limits_(const double &f0, const double &f1, const double &f2,
                                 const double &max_force_norm, const double &t0, const double &t1,
                                 const double &t2, const double &max_torque) const;

  LBRWrenchCommandControllerParameters parameters_;

  lbr_fri_ros2::jnt_name_array_t joint_names_;

  // referenced by state interfaces
  lbr_fri_ros2::jnt_array_t joint_position_states_;
  lbr_fri_ros2::jnt_array_t joint_velocity_states_;

  // state interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_, joint_velocity_state_interfaces_;

  // command interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      joint_position_command_interfaces_, wrench_command_interfaces_;

  // in external mode, wrench and joint position are commanded
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr>
      lbr_wrench_command_rt_buffer_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRWrenchCommand>::SharedPtr
      lbr_wrench_command_subscription_ptr_;

  // in chained mode, only wrench is commanded
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Wrench::SharedPtr> wrench_command_rt_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_command_subscription_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_
