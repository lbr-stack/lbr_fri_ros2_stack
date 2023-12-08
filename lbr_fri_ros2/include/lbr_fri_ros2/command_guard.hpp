#ifndef LBR_FRI_ROS2__COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRClient.h"
#include "friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"

namespace lbr_fri_ros2 {
struct CommandGuardParameters {
  // ROS IDL types
  using jnt_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using jnt_name_array_t = std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

  jnt_name_array_t joint_names;                         /**< Joint names.*/
  jnt_array_t min_position{0., 0., 0., 0., 0., 0., 0.}; /**< Minimum joint position [rad].*/
  jnt_array_t max_position{0., 0., 0., 0., 0., 0., 0.}; /**< Maximum joint position [rad].*/
  jnt_array_t max_velocity{0., 0., 0., 0., 0., 0., 0.}; /**< Maximum joint velocity [rad/s].*/
  jnt_array_t max_torque{0., 0., 0., 0., 0., 0., 0.};   /**< Maximum joint torque [Nm].*/
};

class CommandGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandGuard";

  // ROS IDL types
  using idl_command_t = lbr_fri_msgs::msg::LBRCommand;
  using const_idl_command_t_ref = const idl_command_t &;
  using jnt_array_t = idl_command_t::_joint_position_type;
  using const_jnt_array_t_ref = const jnt_array_t &;

  // FRI types
  using fri_state_t = KUKA::FRI::LBRState;
  using const_fri_state_t_ref = const fri_state_t &;

public:
  CommandGuard() = default;
  CommandGuard(const CommandGuardParameters &command_guard_parameters);
  virtual bool is_valid_command(const_idl_command_t_ref lbr_command,
                                const_fri_state_t_ref lbr_state) const;

  void log_info() const;

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_fri_state_t_ref /*lbr_state*/) const;
  virtual bool command_in_velocity_limits_(const_idl_command_t_ref lbr_command,
                                           const_fri_state_t_ref lbr_state) const;
  virtual bool command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                         const_fri_state_t_ref lbr_state) const;

  CommandGuardParameters parameters_;
};

class SafeStopCommandGuard : public CommandGuard {
public:
  SafeStopCommandGuard(const CommandGuardParameters &command_guard_parameters)
      : CommandGuard(command_guard_parameters){};

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_fri_state_t_ref lbr_state) const override;
};

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant = "default");
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_GUARD_HPP_
