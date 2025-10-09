#ifndef LBR_FRI_ROS2__INTERFACES__TORQUE_ONLY_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__TORQUE_ONLY_COMMAND_HPP_

#include <algorithm>

#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {
class TorqueOnlyCommandInterface : public BaseCommandInterface {
protected:
  std::string LOGGER_NAME() const override { return "lbr_fri_ros2::TorqueCommandInterface"; }
private:
    int init_state;  // interface loop state:
                     // 1 - the first run,
                     // 2 - no control received yet,
                     // 0 - accepting torque control inputs
    double init_pos[7]; // initial joint pos held by internal P-controller when init_state > 0

public:
  TorqueOnlyCommandInterface() = delete;
  TorqueOnlyCommandInterface(const double &joint_position_tau,
                         const CommandGuardParameters &command_guard_parameters,
                         const std::string &command_guard_variant = "default");

  void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) override;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__TORQUE_ONLY_COMMAND_HPP_
