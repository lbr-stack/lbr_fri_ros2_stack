#include "lbr_fri_ros2/command_interface.hpp"

namespace lbr_fri_ros2 {
void CommandInterface::get_command(fri_command_type_ref command, const_fri_state_type_ref state) {
  if (state.getSessionState() != KUKA::FRI::ESessionState::COMMANDING_WAIT &&
      state.getSessionState() != KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    init_command_(state);
    command.setJointPosition(command_.joint_position.data());
    command.setTorque(command_.torque.data());
    command.setWrench(command_.wrench.data());
    return;
  }

  if (state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT) {
    init_command_(state);
    command.setJointPosition(command_.joint_position.data());
    command.setTorque(command_.torque.data());
    command.setWrench(command_.wrench.data());
    return;
  }

  // TODO: add PID etc

  command.setJointPosition(state.getMeasuredJointPosition());
}

void CommandInterface::init_command_(const_fri_state_type_ref state) {
  std::memcpy(command_target_.joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  command_ = command_target_;
  init_ = true;
}
} // namespace lbr_fri_ros2