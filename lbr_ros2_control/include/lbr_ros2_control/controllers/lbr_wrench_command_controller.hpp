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
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "semantic_components/force_torque_sensor.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRWrenchCommandController : public controller_interface::ChainableControllerInterface {
  static constexpr uint8_t CARTESIAN_DOF = 6;

public:
  LBRWrenchCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  // expect full lbr_wrench command in this mode....
  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

  // expect just wrench command in this mode....
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool reference_state_interfaces_();
  bool reference_command_interfaces_();
  void clear_state_interfaces_();
  void clear_command_interfaces_();
  void configure_joint_names_();
  void init_lbr_wrench_command_subscription_();
  void init_wrench_command_subscription_();
  void reset_lbr_wrench_command_subscription_();
  void reset_wrench_command_subscription_();

  lbr_fri_ros2::jnt_name_array_t joint_names_;

  // referenced by state interfaces
  lbr_fri_ros2::jnt_array_t joint_position_states_;
  lbr_fri_ros2::jnt_array_t joint_velocity_states_;

  // state interfaces, consider access to external force interface for safety checking....
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_, joint_velocity_state_interfaces_;

  // make use of the estimated force-torque sensor state interface to read externally applied
  // forces. The forces are used to verify the robot's load data was calibrated
  std::unique_ptr<semantic_components::ForceTorqueSensor> estimated_ft_sensor_ptr_;

  // command interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      joint_position_command_interfaces_, wrench_command_interfaces_;

  // in external mode, wrench and joint position are commanded
  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr>
      rt_lbr_wrench_command_ptr_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRWrenchCommand>::SharedPtr
      lbr_wrench_command_subscription_ptr_;

  // in chained mode, only wrench is commanded
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Wrench::SharedPtr> rt_wrench_command_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_command_subscription_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_WRENCH_COMMAND_CONTROLLER_HPP_
