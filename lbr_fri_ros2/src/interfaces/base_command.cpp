#include "lbr_fri_ros2/interfaces/base_command.hpp"

namespace lbr_fri_ros2 {

BaseCommandInterface::BaseCommandInterface(const double &joint_position_tau,
                                           const CommandGuardParameters &command_guard_parameters,
                                           const std::string &command_guard_variant)
    : command_initialized_(false), joint_position_filter_(joint_position_tau) {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
};

void BaseCommandInterface::init_command(const_idl_state_t_ref state) {
  neutralize_command_(state, command_target_);
  command_ = command_target_;
  command_initialized_ = true;
}

void BaseCommandInterface::log_info() const {
  command_guard_->log_info();
  joint_position_filter_.log_info();
}

void BaseCommandInterface::neutralize_command_(const_idl_state_t_ref state,
                                               idl_command_t_ref command) {
  command.joint_position = state.measured_joint_position;
  command.torque.fill(0.);
  command.wrench.fill(0.);
}
} // namespace lbr_fri_ros2
