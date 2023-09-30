#ifndef LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_
#define LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_

#include <memory>

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
// #include "lbr_fri_ros2/command_guard.hpp"
#include "lbr_fri_ros2/utils.hpp"

namespace lbr_fri_ros2 {
class CommandInterface {
protected:
  // ROS types
  using ros_command_type = lbr_fri_msgs::msg::LBRCommand;
  using const_ros_command_type_ref = const ros_command_type &;

  // FRI types
  using fri_command_type = KUKA::FRI::LBRCommand;
  using fri_command_type_ref = fri_command_type &;
  using fri_state_type = KUKA::FRI::LBRState;
  using const_fri_state_type_ref = const fri_state_type &;

public:
  CommandInterface() = default;

  inline bool is_init() const { return init_; };

  void get_command(fri_command_type_ref command, const_fri_state_type_ref state);
  inline void set_command(const_ros_command_type_ref command) { command_ = command; };

protected:
  void init_command_(const_fri_state_type_ref state);
  // std::unique_ptr<CommandGuard> command_guard_;
  // add PID
  ros_command_type command_, command_target_;
  bool init_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_INTERFACE_HPP_
