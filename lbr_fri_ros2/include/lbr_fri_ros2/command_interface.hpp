#ifndef LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_
#define LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/filters.hpp"

namespace lbr_fri_ros2 {
class CommandInterface {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandInterface";

  // ROS IDL types
  using idl_command_t = lbr_fri_msgs::msg::LBRCommand;
  using const_idl_command_t_ref = const idl_command_t &;

  // FRI types
  using fri_command_t = KUKA::FRI::LBRCommand;
  using fri_command_t_ref = fri_command_t &;
  using fri_state_t = KUKA::FRI::LBRState;
  using const_fri_state_t_ref = const fri_state_t &;

public:
  CommandInterface() = delete;
  CommandInterface(const PIDParameters &pid_parameters,
                   const CommandGuardParameters &command_guard_parameters,
                   const std::string &command_guard_variant = "default");

  void get_joint_position_command(fri_command_t_ref command, const_fri_state_t_ref state);
  void get_torque_command(fri_command_t_ref command, const_fri_state_t_ref state);
  void get_wrench_command(fri_command_t_ref command, const_fri_state_t_ref state);

  void init_command(const_fri_state_t_ref state);
  inline void set_command_target(const_idl_command_t_ref command) { command_target_ = command; }
  inline const_idl_command_t_ref get_command() const { return command_; }
  inline const_idl_command_t_ref get_command_target() const { return command_target_; }

  void log_info() const;

protected:
  std::unique_ptr<CommandGuard> command_guard_;
  PIDParameters pid_parameters_;
  JointPIDArray joint_position_pid_;
  idl_command_t command_, command_target_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_
