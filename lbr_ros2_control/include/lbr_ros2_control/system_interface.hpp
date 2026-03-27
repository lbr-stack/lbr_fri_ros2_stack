#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_

#include <algorithm>
#include <array>
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
  struct Parameters {
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

  struct CommandInterfaceHandles {
    std::array<hardware_interface::CommandInterface::SharedPtr, lbr_fri_ros2::N_JNTS>
        joint_position, torque;
    std::array<hardware_interface::CommandInterface::SharedPtr, lbr_fri_ros2::CARTESIAN_DOF> wrench;

    void populate(const hardware_interface::SystemInterface &si) {
      const auto &info = si.get_hardware_info();
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        const auto &joint_name = info.joints[i].name;
        joint_position[i] =
            si.get_command_interface_handle(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        torque[i] =
            si.get_command_interface_handle(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      }
      wrench[0] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_X);
      wrench[1] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Y);
      wrench[2] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Z);
      wrench[3] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_X);
      wrench[4] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Y);
      wrench[5] =
          si.get_command_interface_handle(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Z);
    }

    void nan_interfaces() const {
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        std::ignore = joint_position[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = torque[i]->set_value(std::numeric_limits<double>::quiet_NaN());
      }
      for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
        std::ignore = wrench[i]->set_value(std::numeric_limits<double>::quiet_NaN());
      }
    }

    void pull(lbr_fri_idl::msg::LBRCommand &lbr_command) const {
      // populate command message
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        lbr_command.joint_position[i] = joint_position[i]->get_optional().value();
        lbr_command.torque[i] = torque[i]->get_optional().value();
      }
      for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
        lbr_command.wrench[i] = wrench[i]->get_optional().value();
      }
    }
  };

  struct StateInterfaceHandles {
#if FRI_CLIENT_VERSION_MAJOR == 1
    std::array<hardware_interface::StateInterface::SharedPtr, lbr_fri_ros2::N_JNTS>
        commanded_joint_position;
#endif
    std::array<hardware_interface::StateInterface::SharedPtr, lbr_fri_ros2::N_JNTS>
        commanded_torque, ipo_joint_position, position, external_torque, effort, velocity;
    hardware_interface::StateInterface::SharedPtr sample_time, session_state, connection_quality,
        safety_state, operation_mode, drive_state, client_command_mode, overlay_type, control_mode,
        time_stamp_sec, time_stamp_nano_sec, tracking_performance;

    void populate(const hardware_interface::SystemInterface &si) {
      const auto &info = si.get_hardware_info();
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        const auto &joint_name = info.joints[i].name;
#if FRI_CLIENT_VERSION_MAJOR == 1
        commanded_joint_position[i] =
            si.get_state_interface_handle(joint_name + "/" + HW_IF_COMMANDED_JOINT_POSITION);
#endif
        commanded_torque[i] =
            si.get_state_interface_handle(joint_name + "/" + HW_IF_COMMANDED_TORQUE);
        ipo_joint_position[i] =
            si.get_state_interface_handle(joint_name + "/" + HW_IF_IPO_JOINT_POSITION);
        position[i] =
            si.get_state_interface_handle(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        external_torque[i] =
            si.get_state_interface_handle(joint_name + "/" + HW_IF_EXTERNAL_TORQUE);
        effort[i] =
            si.get_state_interface_handle(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
        velocity[i] =
            si.get_state_interface_handle(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      }

      sample_time = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                  HW_IF_SAMPLE_TIME);
      session_state = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                    HW_IF_SESSION_STATE);
      connection_quality = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                         HW_IF_CONNECTION_QUALITY);
      safety_state = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                   HW_IF_SAFETY_STATE);
      operation_mode = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                     HW_IF_OPERATION_MODE);
      drive_state = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                  HW_IF_DRIVE_STATE);
      client_command_mode = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) +
                                                          "/" + HW_IF_CLIENT_COMMAND_MODE);
      overlay_type = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                   HW_IF_OVERLAY_TYPE);
      control_mode = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                   HW_IF_CONTROL_MODE);
      time_stamp_sec = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                                     HW_IF_TIME_STAMP_SEC);
      time_stamp_nano_sec = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) +
                                                          "/" + HW_IF_TIME_STAMP_NANO_SEC);
      tracking_performance = si.get_state_interface_handle(std::string(HW_IF_AUXILIARY_PREFIX) +
                                                           "/" + HW_IF_TRACKING_PERFORMANCE);
    }

    void nan_interfaces() const {
      // joint state interfaces
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        std::ignore = position[i]->set_value(std::numeric_limits<double>::quiet_NaN());
#if FRI_CLIENT_VERSION_MAJOR == 1
        std::ignore =
            commanded_joint_position[i]->set_value(std::numeric_limits<double>::quiet_NaN());
#endif
        std::ignore = effort[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = commanded_torque[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = external_torque[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = ipo_joint_position[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = velocity[i]->set_value(std::numeric_limits<double>::quiet_NaN());
      }
      std::ignore = sample_time->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = tracking_performance->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = session_state->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = connection_quality->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = safety_state->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = operation_mode->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = drive_state->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = client_command_mode->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = overlay_type->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = control_mode->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = time_stamp_sec->set_value(std::numeric_limits<double>::quiet_NaN());
      std::ignore = time_stamp_nano_sec->set_value(std::numeric_limits<double>::quiet_NaN());
    }

    void
    push(const lbr_fri_idl::msg::LBRState &lbr_state,
         const lbr_fri_idl::msg::LBRState::_measured_joint_position_type &velocity_estimate) const {
      // set the joint state interfaces
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
#if FRI_CLIENT_VERSION_MAJOR == 1
        std::ignore = commanded_joint_position[i]->set_value(lbr_state.commanded_joint_position[i]);
#endif
        std::ignore = commanded_torque[i]->set_value(lbr_state.commanded_torque[i]);
        std::ignore = ipo_joint_position[i]->set_value(lbr_state.ipo_joint_position[i]);
        std::ignore = position[i]->set_value(lbr_state.measured_joint_position[i]);
        std::ignore = external_torque[i]->set_value(lbr_state.external_torque[i]);
        std::ignore = effort[i]->set_value(lbr_state.measured_torque[i]);
        std::ignore = velocity[i]->set_value(velocity_estimate[i]);
      }

      // state interfaces without cast
      std::ignore = sample_time->set_value(lbr_state.sample_time);
      std::ignore = tracking_performance->set_value(lbr_state.tracking_performance);

      // state interfaces with cast
      std::ignore = session_state->set_value(static_cast<double>(lbr_state.session_state));
      std::ignore =
          connection_quality->set_value(static_cast<double>(lbr_state.connection_quality));
      std::ignore = safety_state->set_value(static_cast<double>(lbr_state.safety_state));
      std::ignore = operation_mode->set_value(static_cast<double>(lbr_state.operation_mode));
      std::ignore = drive_state->set_value(static_cast<double>(lbr_state.drive_state));
      std::ignore =
          client_command_mode->set_value(static_cast<double>(lbr_state.client_command_mode));
      std::ignore = overlay_type->set_value(static_cast<double>(lbr_state.overlay_type));
      std::ignore = control_mode->set_value(static_cast<double>(lbr_state.control_mode));
      std::ignore = time_stamp_sec->set_value(static_cast<double>(lbr_state.time_stamp_sec));
      std::ignore =
          time_stamp_nano_sec->set_value(static_cast<double>(lbr_state.time_stamp_nano_sec));
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
  Parameters parameters_;

  // robot driver
  std::shared_ptr<lbr_fri_ros2::AsyncClient> async_client_ptr_;
  std::unique_ptr<lbr_fri_ros2::App> app_ptr_;

  // session state tracking
  KUKA::FRI::ESessionState previous_session_state_;

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

  // interface handles for commands / states
  CommandInterfaceHandles command_if_handles_;
  StateInterfaceHandles state_if_handles_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_HPP_
