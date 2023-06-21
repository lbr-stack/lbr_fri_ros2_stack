#ifndef LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_
#define LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_

#include <cstring>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/lbr_app.hpp"
#include "lbr_hardware_interface/lbr_hardware_interface_type_values.hpp"

namespace lbr_hardware_interface {
class LBRHardwareInterface
    : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
  LBRHardwareInterface() = default;

  // hardware interface
  hardware_interface::return_type configure(
      const hardware_interface::HardwareInfo &info) override; // check ros2 control and set status
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override; // not supported in FRI

  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

protected:
  template <typename ServiceT>
  bool wait_for_service_(const typename rclcpp::Client<ServiceT>::SharedPtr client,
                         const uint8_t &attempts = 10,
                         const std::chrono::seconds &timeout = std::chrono::seconds(1));

  // setup
  void init_command_interfaces_();
  void init_state_interfaces_();
  bool verify_number_of_joints_();
  bool verify_joint_command_interfaces_();
  bool verify_joint_state_interfaces_();
  bool verify_sensors_();
  bool spawn_com_layer_();
  bool spawn_clients_();

  // monitor end of commanding active
  bool exit_commanding_active_(const KUKA::FRI::ESessionState &previous_session_state,
                               const KUKA::FRI::ESessionState &session_state);

  // connect to - disconnect from robot
  bool connect_();
  bool disconnect_();

  void on_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state);

  const uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 7;
  const uint8_t LBR_FRI_COMMAND_INTERFACE_SIZE = 2;
  const uint8_t LBR_FRI_SENSOR_SIZE = 12;

  // node for handling communication
  rclcpp::Node::SharedPtr hw_node_;
  rclcpp::Node::SharedPtr lbr_app_node_;
  std::unique_ptr<lbr_fri_ros2::LBRApp> lbr_app_;

  // exposed state interfaces
  double hw_sample_time_;
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

  lbr_fri_msgs::msg::LBRState::_measured_joint_position_type hw_position_;
  lbr_fri_msgs::msg::LBRState::_commanded_joint_position_type hw_commanded_joint_position_;
  lbr_fri_msgs::msg::LBRState::_measured_torque_type hw_effort_;
  lbr_fri_msgs::msg::LBRState::_commanded_torque_type hw_commanded_torque_;
  lbr_fri_msgs::msg::LBRState::_external_torque_type hw_external_torque_;
  lbr_fri_msgs::msg::LBRState::_ipo_joint_position_type hw_ipo_joint_position_;
  double hw_tracking_performance_;

  // comput velocity for state interface
  double time_stamps_to_sec_(const double &sec, const double &nano_sec) const;
  void init_last_hw_states_();
  void update_last_hw_states_();
  void compute_hw_velocity_();

  lbr_fri_msgs::msg::LBRState::_measured_joint_position_type last_hw_position_;
  double last_hw_time_stamp_sec_;
  double last_hw_time_stamp_nano_sec_;
  lbr_fri_msgs::msg::LBRState::_measured_joint_position_type hw_velocity_;

  // exposed command interfaces
  lbr_fri_msgs::msg::LBRCommand::_joint_position_type hw_position_command_;
  lbr_fri_msgs::msg::LBRCommand::_torque_type hw_effort_command_;

  // app connect call request
  int32_t port_id_;
  const char *remote_host_;
  uint8_t sample_time_;

  // publisher for sending commands / subscriber to receive goals
  lbr_fri_msgs::msg::LBRCommand lbr_command_;
  lbr_fri_msgs::msg::LBRState lbr_state_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;

  // clients to connect to / disconnect from LBR
  rclcpp::Client<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_clt_;
  rclcpp::Client<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_clt_;

  std::unique_ptr<std::thread> node_thread_;
};
} // end of namespace lbr_hardware_interface
#endif // LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_
