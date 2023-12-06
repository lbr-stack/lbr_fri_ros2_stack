#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_

#include <algorithm>
#include <cstring>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/client.hpp"
#include "lbr_fri_ros2/enum_maps.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class SystemInterface : public hardware_interface::SystemInterface {
  static constexpr uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 7;
  static constexpr uint8_t LBR_FRI_COMMAND_INTERFACE_SIZE = 2;
  static constexpr uint8_t LBR_FRI_SENSOR_SIZE = 12;

public:
  SystemInterface() = default;

  // hardware interface
  controller_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override; // not supported in FRI

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

protected:
  // setup
  void nan_command_interfaces_();
  void nan_state_interfaces_();
  bool verify_number_of_joints_();
  bool verify_joint_command_interfaces_();
  bool verify_joint_state_interfaces_();
  bool verify_sensors_();

  // monitor end of commanding active
  bool exit_commanding_active_(const KUKA::FRI::ESessionState &previous_session_state,
                               const KUKA::FRI::ESessionState &session_state);

  // robot parameters
  int32_t port_id_;
  const char *remote_host_;
  int32_t rt_prio_;
  bool open_loop_;

  // robot driver
  rclcpp::Node::SharedPtr app_node_ptr_;
  std::shared_ptr<lbr_fri_ros2::Client> client_ptr_;
  std::unique_ptr<lbr_fri_ros2::App> app_ptr_;

  // exposed state interfaces
  lbr_fri_msgs::msg::LBRState hw_lbr_state_;

  // state interfaces that require cast, this could be mitigated by defining LBRState exclusively
  // with doubles
  double hw_session_state_;
  double hw_connection_quality_;
  double hw_safety_state_;
  double hw_operation_mode_;
  double hw_drive_state_;
  double hw_client_command_mode_;
  double hw_overlay_type_;
  double hw_control_mode_;
  double hw_time_stamp_sec_;
  double hw_time_stamp_nano_sec_;

  // added velocity state interface
  lbr_fri_msgs::msg::LBRState::_measured_joint_position_type last_hw_measured_joint_position_;
  double last_hw_time_stamp_sec_;
  double last_hw_time_stamp_nano_sec_;
  lbr_fri_msgs::msg::LBRState::_measured_joint_position_type hw_velocity_;

  // compute velocity for state interface
  double time_stamps_to_sec_(const double &sec, const double &nano_sec) const;
  void nan_last_hw_states_();
  void update_last_hw_states_();
  void compute_hw_velocity_();

  // exposed command interfaces
  lbr_fri_msgs::msg::LBRCommand hw_lbr_command_;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
