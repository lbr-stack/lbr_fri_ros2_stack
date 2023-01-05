#ifndef LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_
#define LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_

#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/lbr.hpp"
#include "lbr_hardware_interface/lbr_hardware_interface_type_values.hpp"

namespace lbr_hardware_interface {
class LBRHardwareInterface : public hardware_interface::SystemInterface {
public:
  LBRHardwareInterface() = default;

  // hardware interface
  controller_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override; // check ros2 control and set status
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
  template <typename ServiceT>
  bool wait_for_service_(const typename rclcpp::Client<ServiceT>::SharedPtr client,
                         const uint8_t &attempts = 10,
                         const std::chrono::seconds &timeout = std::chrono::seconds(1));

  // setup
  bool init_lbr_();
  bool init_command_interfaces_();
  bool init_state_interfaces_();
  bool verify_number_of_joints_();
  bool verify_joint_command_interfaces_();
  bool verify_joint_state_interfaces_();
  bool verify_sensors_();
  bool spawn_rt_layer_();
  bool spawn_clients_();

  // connect to - disconnect from robot
  bool connect_();
  bool disconnect_();

  void lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state);

  const uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 7;
  const uint8_t LBR_FRI_COMMAND_INTERFACE_SIZE = 2;
  const uint8_t LBR_FRI_SENSOR_SIZE = 12;

  // node for handling communication
  rclcpp::Node::SharedPtr node_;

  // intermadiate buffer
  std::unique_ptr<lbr_fri_ros2::LBR> lbr_;

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

  std::vector<double> hw_position_;
  std::vector<double> hw_commanded_joint_position_;
  std::vector<double> hw_effort_;
  std::vector<double> hw_commanded_torque_;
  std::vector<double> hw_external_torque_;
  std::vector<double> hw_ipo_joint_position_;
  double hw_tracking_performance_;

  // comput velocity for state interface
  double time_stamps_to_sec_(const double &sec, const double &nano_sec) const;
  bool init_last_hw_states_();
  bool update_last_hw_states_();
  void compute_hw_velocity_();

  std::vector<double> last_hw_position_;
  double last_hw_time_stamp_sec_;
  double last_hw_time_stamp_nano_sec_;
  std::vector<double> hw_velocity_;

  // exposed command interfaces
  std::vector<double> hw_position_command_;
  std::vector<double> hw_effort_command_;

  // app connect call request
  int32_t port_id_;
  const char *remote_host_;

  // publisher for sending commands / subscriber to receive goals
  std::shared_ptr<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRState::SharedPtr>>
      rt_lbr_state_buf_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRCommand>>
      rt_lbr_command_pub_;

  // clients to switch controllers (to be replaced by ERROR return in read/write,
  // see https://discourse.ros.org/t/ros2-control-controller-restart/24662,
  // https://github.com/ros-controls/ros2_control/issues/674, and
  // https://github.com/ros-controls/ros2_control/pull/677)
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_clt_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_clt_;

  // clients to connect to / disconnect from LBR
  rclcpp::Client<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_clt_;
  rclcpp::Client<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_clt_;

  std::unique_ptr<std::thread> node_thread_;
};
} // end of namespace lbr_hardware_interface
#endif // LBR_HARDWARE_INTERFACE__LBR_HARDWARE_INTERFACE_HPP_
