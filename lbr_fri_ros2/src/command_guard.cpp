#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {
CommandGuard::CommandGuard(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr)
    : logging_interface_ptr_(logging_interface_ptr){};

CommandGuard::CommandGuard(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
    const_joint_array_t_ref min_position, const_joint_array_t_ref max_position,
    const_joint_array_t_ref max_velocity, const_joint_array_t_ref max_torque)
    : logging_interface_ptr_(logging_interface_ptr), min_position_(min_position),
      max_position_(max_position), max_velocity_(max_velocity), max_torque_(max_torque) {}

CommandGuard::CommandGuard(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
    const std::string &robot_description)
    : logging_interface_ptr_(logging_interface_ptr) {
  urdf::Model model;
  if (!model.initString(robot_description)) {
    std::string err = "Failed to intialize urdf model from robot description.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
  std::size_t jnt_cnt = 0;
  for (const auto &name_joint_pair : model.joints_) {
    const auto joint = name_joint_pair.second;
    if (joint->type == urdf::Joint::REVOLUTE) {
      if (jnt_cnt > std::tuple_size<joint_array_t>()) {
        std::string errs = "Found too many joints in robot description.";
        RCLCPP_ERROR(logging_interface_ptr_->get_logger(), errs.c_str());
        throw std::runtime_error(errs);
      }
      min_position_[jnt_cnt] = joint->limits->lower;
      max_position_[jnt_cnt] = joint->limits->upper;
      max_velocity_[jnt_cnt] = joint->limits->velocity;
      max_torque_[jnt_cnt] = joint->limits->effort;
      ++jnt_cnt;
    }
  }
  if (jnt_cnt != std::tuple_size<joint_array_t>()) {
    std::string err = "Didn't find expected number of joints in robot description.";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  };
}

bool CommandGuard::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_fri_state_t_ref lbr_state) const {
  switch (lbr_state.getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return false;
  case KUKA::FRI::EClientCommandMode::POSITION:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command, lbr_state)) {
      return false;
    }
    if (!command_in_velocity_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (is_nan_(lbr_command.wrench.cbegin(), lbr_command.wrench.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    if (is_nan_(lbr_command.joint_position.cbegin(), lbr_command.joint_position.cend())) {
      return false;
    }
    if (is_nan_(lbr_command.torque.cbegin(), lbr_command.torque.cend())) {
      return false;
    }
    if (!command_in_position_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
  default:
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Invalid EClientCommandMode provided.");
    return false;
  }

  return true;
}

bool CommandGuard::is_nan_(const double *begin, const double *end) const {
  return std::find_if(begin, end, [&](const auto &xi) { return std::isnan(xi); }) != end;
}

bool CommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                               const_fri_state_t_ref /*lbr_state*/) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] < min_position_[i] ||
        lbr_command.joint_position[i] > max_position_[i]) {
      RCLCPP_ERROR(logging_interface_ptr_->get_logger(),
                   "Position command not in limits for joint A%ld.", i + 1);
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_velocity_limits_(const_idl_command_t_ref lbr_command,
                                               const_fri_state_t_ref lbr_state) const {
  const double &dt = lbr_state.getSampleTime();
  for (std::size_t i = 0; i < lbr_command.joint_position[i]; ++i) {
    if (std::abs(lbr_command.joint_position[i] - lbr_state.getMeasuredJointPosition()[i]) / dt >
        max_velocity_[i]) {
      RCLCPP_ERROR(logging_interface_ptr_->get_logger(), "Velocity not in limits for joint A%ld.",
                   i + 1);
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                             const_fri_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
    if (std::abs(lbr_command.torque[i] + lbr_state.getExternalTorque()[i]) > max_torque_[i]) {
      RCLCPP_ERROR(logging_interface_ptr_->get_logger(),
                   "Torque command not in limits for joint A%ld.", i + 1);
      return false;
    }
  }
  return true;
}

bool SafeStopCommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                                       const_fri_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] <
            min_position_[i] + max_velocity_[i] * lbr_state.getSampleTime() ||
        lbr_command.joint_position[i] >
            max_position_[i] - max_velocity_[i] * lbr_state.getSampleTime()) {
      RCLCPP_ERROR(logging_interface_ptr_->get_logger(),
                   "Position command not in limits for joint A%ld.", i + 1);
      return false;
    }
  }
  return true;
}

std::unique_ptr<CommandGuard> command_guard_factory(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
    const std::string &robot_description, const std::string &variant) {
  if (variant == "default") {
    return std::make_unique<CommandGuard>(logging_interface_ptr, robot_description);
  }
  if (variant == "safe_stop") {
    return std::make_unique<SafeStopCommandGuard>(logging_interface_ptr, robot_description);
  }
  std::string err = "Invalid CommandGuard variant provided.";
  RCLCPP_ERROR(logging_interface_ptr->get_logger(), err.c_str());
  throw std::runtime_error(err);
}
} // end of namespace lbr_fri_ros2
