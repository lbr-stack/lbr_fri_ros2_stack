#ifndef LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
#define LBR_FRI_ROS2__INTERACES__COMMAND_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/guards/command_guard.hpp"
#include "lbr_fri_ros2/triple_buffer.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
class BaseCommandInterface {
protected:
  virtual std::string LOGGER_NAME() const = 0;

public:
  BaseCommandInterface() = delete;
  BaseCommandInterface(const double &joint_position_tau,
                       const CommandGuardParameters &command_guard_parameters,
                       const std::string &command_guard_variant = "default");

  /**
   * @brief Write a command target for the FRI thread. Real-time safe, never blocks. Called from
   * the controller_manager thread only (single producer).
   *
   * @param[in] command The command target.
   */
  void buffer_command_target(const_idl_command_t_ref command) {
    command_target_buffer_.write(command);
  }

  /**
   * @brief Send the latest command target to the robot. FRI thread only.
   */
  virtual void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) = 0;

  /**
   * @brief Reset the command to the current state. FRI thread only.
   */
  void init_command(const_idl_state_t_ref state);

  /**
   * @brief The command last sent to the robot. FRI thread only.
   */
  inline const_idl_command_t_ref get_command() const { return command_; }

  void log_info() const;

protected:
  /**
   * @brief Copy a newly buffered command target into #command_target_. Leaves #command_target_
   * untouched if nothing was written since the last pull, so a reset by #init_command is not
   * overridden by an old target. FRI thread only.
   */
  void pull_command_target_();
  void neutralize_command_(const_idl_state_t_ref state, idl_command_t_ref command);

protected:
  // shared between threads: written by the controller_manager thread, read by the FRI thread
  TripleBuffer<idl_command_t> command_target_buffer_;

  // FRI thread only
  bool command_initialized_;
  std::unique_ptr<CommandGuard> command_guard_;
  ExponentialFilterArray<N_JNTS> joint_position_filter_;
  idl_command_t command_, command_target_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
