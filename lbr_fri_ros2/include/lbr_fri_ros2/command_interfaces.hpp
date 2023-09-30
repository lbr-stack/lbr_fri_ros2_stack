#ifndef LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_
#define LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_

#include <memory>

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_torque_command.hpp"
#include "lbr_fri_msgs/msg/lbr_wrench_command.hpp"
// #include "lbr_fri_ros2/command_guards.hpp"

namespace lbr_fri_ros2 {
template <typename ros_command> class CommandInterface {
protected:
  // ROS types
  using const_ros_command_ref = const ros_command &;

  // FRI types
  using fri_command = KUKA::FRI::LBRCommand;
  using fri_command_ref = fri_command &;

public:
  CommandInterface() = default;

  virtual void get_command(fri_command_ref command) const = 0;
  void set_command(const_ros_command_ref command) { command_ = command; };

protected:
  // std::unique_ptr<CommandGuard<ros_command>> command_guard_;
  ros_command command_;
};

class PositionCommandInterface : public CommandInterface<lbr_fri_msgs::msg::LBRPositionCommand> {
public:
  PositionCommandInterface() = default;
  void get_command(fri_command_ref command) const override;
};

class TorqueCommandInterface : public CommandInterface<lbr_fri_msgs::msg::LBRTorqueCommand> {
public:
  TorqueCommandInterface() = default;

  void get_command(fri_command_ref command) const override;
};

class WrenchCommandInterface : public CommandInterface<lbr_fri_msgs::msg::LBRWrenchCommand> {
public:
  WrenchCommandInterface() = default;

  void get_command(fri_command_ref command) const override;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_