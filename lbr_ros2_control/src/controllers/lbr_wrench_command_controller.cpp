#include "lbr_ros2_control/controllers/lbr_wrench_command_controller.hpp"

namespace lbr_ros2_control {
LBRWrenchCommandController::LBRWrenchCommandController()
    : lbr_wrench_command_rt_buffer_(nullptr), lbr_wrench_command_subscription_ptr_(nullptr),
      wrench_command_rt_buffer_(nullptr), wrench_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRWrenchCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_X);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Y);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_FORCE_Z);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_X);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Y);
  interface_configuration.names.push_back(std::string(HW_IF_WRENCH_PREFIX) + "/" + HW_IF_TORQUE_Z);
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRWrenchCommandController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // joint position and joint velocity interfaces
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn LBRWrenchCommandController::on_init() {
  try {
    init_lbr_wrench_command_subscription_();
    this->get_node()->declare_parameter("robot_name", "lbr");
    this->get_node()->declare_parameter("max_force_command_norm", 10.0);
    this->get_node()->declare_parameter("max_torque_command_norm", 10.0);
    configure_joint_names_();
    configure_parameters_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to initialize LBR wrench command controller with: "
                            << e.what() << "." << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LBRWrenchCommandController::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    state_interfaces.emplace_back(std::string(get_node()->get_name()) + "/" + joint_names_[i],
                                  hardware_interface::HW_IF_POSITION, &joint_position_states_[i]);
    state_interfaces.emplace_back(std::string(get_node()->get_name()) + "/" + joint_names_[i],
                                  hardware_interface::HW_IF_VELOCITY, &joint_velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LBRWrenchCommandController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces_.resize(lbr_fri_ros2::N_JNTS, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    reference_interfaces.emplace_back(std::string(get_node()->get_name()) + "/" + joint_names_[i],
                                      hardware_interface::HW_IF_POSITION,
                                      &reference_interfaces_[i]);
  }
  return reference_interfaces;
}

bool LBRWrenchCommandController::on_set_chained_mode(bool chained_mode) {
  try {
    if (chained_mode) {
      reset_lbr_wrench_command_subscription_();
      init_wrench_command_subscription_();
    } else {
      reset_wrench_command_subscription_();
      init_lbr_wrench_command_subscription_();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to switch to chained mode with: " << e.what() << "."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

controller_interface::return_type
LBRWrenchCommandController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {
  auto lbr_wrench_command = lbr_wrench_command_rt_buffer_.readFromRT();
  if (!lbr_wrench_command || !(*lbr_wrench_command)) {
    return controller_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    reference_interfaces_[i] = (*lbr_wrench_command)->joint_position[i];
  }

  // zero wrenches if received command out of limits
  if (!command_in_wrench_limits_(
          (*lbr_wrench_command)->wrench[0], (*lbr_wrench_command)->wrench[1],
          (*lbr_wrench_command)->wrench[2], parameters_.max_force_command_norm,
          (*lbr_wrench_command)->wrench[3], (*lbr_wrench_command)->wrench[4],
          (*lbr_wrench_command)->wrench[5], parameters_.max_torque_command_norm)) {
    if (!zero_wrench_commands_()) {
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  // set wrenches from command
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {

    if (!wrench_command_interfaces_[i].get().set_value((*lbr_wrench_command)->wrench[i])) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                              << "Failed to set wrench for '" << i
                                                              << "-axis'."
                                                              << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
LBRWrenchCommandController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {
  // pass joint position and velocity states through to next controller
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    auto q_i = joint_position_state_interfaces_[i].get().get_optional();
    auto dq_i = joint_velocity_state_interfaces_[i].get().get_optional();
    if (!q_i.has_value()) {
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get joint position for joint '" << i << "'."
                             << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::OK;
    }
    if (!dq_i.has_value()) {
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get joint velocity for joint '" << i << "'."
                             << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::OK;
    }
    joint_position_states_[i] = *q_i;
    joint_velocity_states_[i] = *dq_i;
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    if (!joint_position_command_interfaces_[i].get().set_value(reference_interfaces_[i])) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Failed to set joint position for joint '" << joint_names_[i]
                              << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    }
  }
  if (!is_in_chained_mode()) {
    return controller_interface::return_type::OK;
  }

  // read wrench command in chained mode
  auto wrench_command = wrench_command_rt_buffer_.readFromRT();
  if (!wrench_command || !(*wrench_command)) {
    if (!zero_wrench_commands_()) {
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  // zero wrenches if received command out of limits
  if (!command_in_wrench_limits_((*wrench_command)->force.x, (*wrench_command)->force.y,
                                 (*wrench_command)->force.z, parameters_.max_force_command_norm,
                                 (*wrench_command)->torque.x, (*wrench_command)->torque.y,
                                 (*wrench_command)->torque.z,
                                 parameters_.max_torque_command_norm)) {
    if (!zero_wrench_commands_()) {
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  // set wrenches from command
  if (!wrench_command_interfaces_[0].get().set_value((*wrench_command)->force.x)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set force for 'x-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  if (!wrench_command_interfaces_[1].get().set_value((*wrench_command)->force.y)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set force for 'y-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  if (!wrench_command_interfaces_[2].get().set_value((*wrench_command)->force.z)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set force for 'z-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  if (!wrench_command_interfaces_[3].get().set_value((*wrench_command)->torque.x)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set torque for 'x-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  if (!wrench_command_interfaces_[4].get().set_value((*wrench_command)->torque.y)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set torque for 'y-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  if (!wrench_command_interfaces_[5].get().set_value((*wrench_command)->torque.z)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Failed to set torque for 'z-axis'."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  reference_interfaces_.assign(lbr_fri_ros2::N_JNTS, std::numeric_limits<double>::quiet_NaN());
  if (!assign_state_interfaces_()) {
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!assign_command_interfaces_()) {
    release_state_interfaces_();
    release_command_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_state_interfaces_();
  release_command_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LBRWrenchCommandController::assign_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.push_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      joint_velocity_state_interfaces_.push_back(std::ref(state_interface));
    }
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint position state interfaces '"
                            << joint_position_state_interfaces_.size()
                            << "' does not match the number of joints "
                               "in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (joint_velocity_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint velocity state interfaces '"
                            << joint_velocity_state_interfaces_.size()
                            << "' does not match the number of joints "
                               "in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

bool LBRWrenchCommandController::assign_command_interfaces_() {
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_command_interfaces_.push_back(std::ref(command_interface));
    }
    if (command_interface.get_prefix_name() == HW_IF_WRENCH_PREFIX) {
      wrench_command_interfaces_.push_back(std::ref(command_interface));
    }
  }
  if (joint_position_command_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint position command interfaces '"
                            << joint_position_command_interfaces_.size()
                            << "' does not match the number of joints "
                               "in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (wrench_command_interfaces_.size() != lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR << "Number of wrench command interfaces '"
                                                         << wrench_command_interfaces_.size()
                                                         << "' does not equal '"
                                                         << lbr_fri_ros2::CARTESIAN_DOF << "'."
                                                         << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void LBRWrenchCommandController::release_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
}

void LBRWrenchCommandController::release_command_interfaces_() {
  joint_position_command_interfaces_.clear();
  wrench_command_interfaces_.clear();
}

void LBRWrenchCommandController::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint names '" << joint_names_.size()
                            << "' does not match the number of joints in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void LBRWrenchCommandController::configure_parameters_() {
  if (this->get_node()->get_parameter("max_force_command_norm").as_double() < 0.0) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Max force command norm parameter must be positive."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure max force parameter.");
  }
  if (this->get_node()->get_parameter("max_torque_command_norm").as_double() < 0.0) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Max torque command norm parameter must be positive."
                            << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure max torque parameter.");
  }
  this->get_node()->get_parameter("max_force_command_norm", parameters_.max_force_command_norm);
  this->get_node()->get_parameter("max_torque_command_norm", parameters_.max_torque_command_norm);
}

bool LBRWrenchCommandController::zero_wrench_commands_() {
  for (std::size_t i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    if (!wrench_command_interfaces_[i].get().set_value(0.0)) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                              << "Failed to zero wrench for '" << i
                                                              << "-axis'."
                                                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

void LBRWrenchCommandController::init_lbr_wrench_command_subscription_() {
  lbr_wrench_command_subscription_ptr_ =
      this->get_node()->create_subscription<lbr_fri_idl::msg::LBRWrenchCommand>(
          "command/lbr_wrench_command", 1,
          [this](const lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr msg) {
            lbr_wrench_command_rt_buffer_.writeFromNonRT(msg);
          });
}

void LBRWrenchCommandController::init_wrench_command_subscription_() {
  wrench_command_subscription_ptr_ =
      this->get_node()->create_subscription<geometry_msgs::msg::Wrench>(
          "command/wrench", 1, [this](const geometry_msgs::msg::Wrench::SharedPtr msg) {
            wrench_command_rt_buffer_.writeFromNonRT(msg);
          });
}

void LBRWrenchCommandController::reset_lbr_wrench_command_subscription_() {
  if (lbr_wrench_command_subscription_ptr_) {
    lbr_wrench_command_subscription_ptr_.reset();
  }
}
void LBRWrenchCommandController::reset_wrench_command_subscription_() {
  if (wrench_command_subscription_ptr_) {
    wrench_command_subscription_ptr_.reset();
  }
}

bool LBRWrenchCommandController::command_in_wrench_limits_(
    const double &f0, const double &f1, const double &f2, const double &max_force_norm,
    const double &t0, const double &t1, const double &t2, const double &max_torque_norm) const {
  // check if force and torque norms are within limits
  if (!lbr_fri_ros2::norm_in_bounds(f0, f1, f2, max_force_norm)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Force command exceeds limit of "
                                                            << max_force_norm << "N."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (!lbr_fri_ros2::norm_in_bounds(t0, t1, t2, max_torque_norm)) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                            << "Torque command exceeds limit of "
                                                            << max_torque_norm << "Nm."
                                                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRWrenchCommandController,
                       controller_interface::ChainableControllerInterface)
