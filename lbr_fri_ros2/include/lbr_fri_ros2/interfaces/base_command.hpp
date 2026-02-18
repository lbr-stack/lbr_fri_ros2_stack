#ifndef LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
#define LBR_FRI_ROS2__INTERACES__COMMAND_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/guards/command_guard.hpp"
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

  virtual void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) = 0;
  void buffer_command_target(const_idl_command_t_ref command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_target_ = command;
  }
  void init_command(const_idl_state_t_ref state);

  idl_command_t get_command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
  }

  void log_info() const;

protected:
  void neutralize_command_(const_idl_state_t_ref state, idl_command_t_ref command);

protected:
  mutable std::mutex command_mutex_;
  bool command_initialized_;
  std::unique_ptr<CommandGuard> command_guard_;
  ExponentialFilterArray<N_JNTS> joint_position_filter_;
  idl_command_t command_, command_target_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERACES__COMMAND_HPP_
