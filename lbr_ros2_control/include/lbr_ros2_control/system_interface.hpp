#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_

#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "friClientVersion.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/guards/command_guard.hpp"
#include "lbr_fri_ros2/guards/state_guard.hpp"
#include "lbr_fri_ros2/interfaces/state.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class SystemInterface : public hardware_interface::SystemInterface {
protected:
  struct SystemInterfaceParameters {
    uint8_t fri_client_sdk_major_version{1};
    uint8_t fri_client_sdk_minor_version{15};
#if FRI_CLIENT_VERSION_MAJOR == 1
    KUKA::FRI::EClientCommandMode client_command_mode{KUKA::FRI::EClientCommandMode::POSITION};
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
    KUKA::FRI::EClientCommandMode client_command_mode{
        KUKA::FRI::EClientCommandMode::JOINT_POSITION};
#endif
    int32_t port_id{30200};
    const char *remote_host{nullptr};
    int32_t rt_prio{80};
    double joint_position_tau{0.04};
    std::string command_guard_variant{"default"};
    bool state_guard_external_torque_safety_check{true};
    double state_guard_external_torque_limit{2.0};
    double external_torque_tau{0.04};
    double measured_torque_tau{0.04};
    bool open_loop{true};
  };

  struct CommandKeys {
    lbr_fri_ros2::jnt_name_array_t joint_position, torque;
    lbr_fri_ros2::cart_name_array_t wrench;

    void populate_keys(const hardware_interface::HardwareInfo &info) {
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        auto joint_name = info.joints[i].name;
        joint_position[i] = joint_name + "/" + hardware_interface::HW_IF_POSITION;
        torque[i] = joint_name + "/" + hardware_interface::HW_IF_EFFORT;
      }
      wrench[0] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_X;
      wrench[1] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Y;
      wrench[2] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Z;
      wrench[3] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_X;
      wrench[4] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Y;
      wrench[5] = std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Z;
    }
  };

  struct StateKeys {
#if FRI_CLIENT_VERSION_MAJOR == 1
    lbr_fri_ros2::jnt_name_array_t commanded_joint_position;
#endif
    lbr_fri_ros2::jnt_name_array_t commanded_torque, ipo_joint_position, position, external_torque,
        effort, velocity;
    std::string sample_time, session_state, connection_quality, safety_state, operation_mode,
        drive_state, client_command_mode, overlay_type, control_mode, time_stamp_sec,
        time_stamp_nano_sec, tracking_performance;

    void populate_keys(const hardware_interface::HardwareInfo &info) {
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        auto joint_name = info.joints[i].name;
#if FRI_CLIENT_VERSION_MAJOR == 1
        commanded_joint_position[i] = joint_name + "/" + HW_IF_COMMANDED_JOINT_POSITION;
#endif
        commanded_torque[i] = joint_name + "/" + HW_IF_COMMANDED_TORQUE;
        ipo_joint_position[i] = joint_name + "/" + HW_IF_IPO_JOINT_POSITION;
        position[i] = joint_name + "/" + hardware_interface::HW_IF_POSITION;
        external_torque[i] = joint_name + "/" + HW_IF_EXTERNAL_TORQUE;
        effort[i] = joint_name + "/" + hardware_interface::HW_IF_EFFORT;
        velocity[i] = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      }

      sample_time = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_SAMPLE_TIME;
      session_state = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_SESSION_STATE;
      connection_quality = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_CONNECTION_QUALITY;
      safety_state = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_SAFETY_STATE;
      operation_mode = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_OPERATION_MODE;
      drive_state = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_DRIVE_STATE;
      client_command_mode = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_CLIENT_COMMAND_MODE;
      overlay_type = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_OVERLAY_TYPE;
      control_mode = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_CONTROL_MODE;
      time_stamp_sec = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_TIME_STAMP_SEC;
      time_stamp_nano_sec = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_TIME_STAMP_NANO_SEC;
      tracking_performance = std::string(HW_IF_AUXILIARY_PREFIX) + "/" + HW_IF_TRACKING_PERFORMANCE;
    }
  };

protected:
#if FRI_CLIENT_VERSION_MAJOR == 1
  static constexpr uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 7;
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
  static constexpr uint8_t LBR_FRI_STATE_INTERFACE_SIZE = 6;
#endif
  static constexpr uint8_t LBR_FRI_COMMAND_INTERFACE_SIZE = 2;
  static constexpr uint8_t LBR_FRI_SENSORS = 1;
  static constexpr uint8_t AUXILIARY_SENSOR_SIZE = 12;
  static constexpr uint8_t GPIO_SIZE = 1;

public:
  SystemInterface() = default;

  // hardware interface
  controller_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override; // not supported in FRI

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

protected:
  // setup
  bool parse_parameters_();
  void nan_command_interfaces_();
  void nan_state_interfaces_();
  bool verify_number_of_joints_();
  bool verify_joint_command_interfaces_();
  bool verify_joint_state_interfaces_();
  bool verify_sensors_();
  bool verify_auxiliary_sensor_();
  bool verify_gpios_();

  // monitor end of commanding active
  bool exit_commanding_active_(const KUKA::FRI::ESessionState &previous_session_state,
                               const KUKA::FRI::ESessionState &session_state);

  // robot parameters
  SystemInterfaceParameters parameters_;

  // robot driver
  std::shared_ptr<lbr_fri_ros2::AsyncClient> async_client_ptr_;
  std::unique_ptr<lbr_fri_ros2::App> app_ptr_;

  // velocity computation
  lbr_fri_idl::msg::LBRState::_measured_joint_position_type last_measured_joint_position_,
      velocity_;
  double last_time_stamp_sec_;
  double last_time_stamp_nano_sec_;

  // compute velocity for state interface
  double time_stamps_to_sec_(const double &sec, const double &nano_sec) const;
  void nan_last_states_();
  void update_last_states_();
  void compute_velocity_();

  // command and state buffers
  lbr_fri_idl::msg::LBRCommand lbr_command_;
  lbr_fri_idl::msg::LBRState lbr_state_;

  // keys for command / state interfaces
  CommandKeys command_keys_;
  StateKeys state_keys_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
