#include "lbr_fri_ros2/command_interfaces.hpp"

namespace lbr_fri_ros2 {
void PositionCommandInterface::get_command(fri_command_ref command) const {
  command.setJointPosition(command_.joint_position.data());
}

void TorqueCommandInterface::get_command(fri_command_ref command) const {
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_.torque.data());
}

void WrenchCommandInterface::get_command(fri_command_ref command) const {
  command.setJointPosition(command_.joint_position.data());
  command.setWrench(command_.wrench.data());
}
} // namespace lbr_fri_ros2