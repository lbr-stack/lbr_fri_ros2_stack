#include "lbr_fri_ros2/guards/state_guard.hpp"

namespace lbr_fri_ros2 {
StateGuard::StateGuard(const StateGuardParameters &state_guard_parameters)
    : parameters_(state_guard_parameters) {};

bool StateGuard::is_valid_state(const_idl_state_t_ref lbr_state) {
  if (!state_in_external_torque_limits_(lbr_state)) {
    return false;
  }
  return true;
}

void StateGuard::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO_STREAM(
      rclcpp::get_logger(LOGGER_NAME),
      "*   External torque safety check (only on activation of compliant control modes): '"
          << (parameters_.external_torque_safety_check ? "true" : "false") << "'");
  for (std::size_t i = 0; i < parameters_.joint_names.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Joint %s limits: External torque: %.1f Nm",
                parameters_.joint_names[i].c_str(), parameters_.max_external_torque[i]);
  }
}

bool StateGuard::state_in_external_torque_limits_(const_idl_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_state.external_torque.size(); ++i) {
    if (std::abs(lbr_state.external_torque[i]) > parameters_.max_external_torque[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "External torque not in limits for joint "
                                             << parameters_.joint_names[i].c_str() << ". Measured: "
                                             << std::abs(lbr_state.external_torque[i])
                                             << " Nm, limit: " << parameters_.max_external_torque[i]
                                             << " Nm" << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}
} // namespace lbr_fri_ros2
