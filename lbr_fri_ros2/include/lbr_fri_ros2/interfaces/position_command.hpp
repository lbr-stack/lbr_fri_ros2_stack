#ifndef LBR_FRI_ROS2__INTERFACES__POSITION_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__POSITION_COMMAND_HPP_

#include <algorithm>

#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {
class PositionCommandInterface : public BaseCommandInterface {
protected:
  std::string LOGGER_NAME() const override { return "lbr_fri_ros2::PositionCommandInterface"; }

public:
  PositionCommandInterface() = delete;
  PositionCommandInterface(const PIDParameters &pid_parameters,
                           const CommandGuardParameters &command_guard_parameters,
                           const std::string &command_guard_variant = "default");

  void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) override;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__POSITION_COMMAND_HPP_
