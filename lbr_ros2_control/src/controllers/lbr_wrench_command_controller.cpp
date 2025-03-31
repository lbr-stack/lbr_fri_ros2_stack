#include "lbr_ros2_control/controllers/lbr_wrench_command_controller.hpp"

namespace lbr_ros2_control {
LBRWrenchCommandController::LBRWrenchCommandController()
    : rt_lbr_wrench_command_ptr_(nullptr), lbr_wrench_command_subscription_ptr_(nullptr),
      rt_wrench_command_ptr_(nullptr), wrench_command_subscription_ptr_(nullptr) {}

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
  // estimated force-torque sensor interface
  for (const auto &interface_name : estimated_ft_sensor_ptr_->get_state_interface_names()) {
    interface_configuration.names.push_back(interface_name);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn LBRWrenchCommandController::on_init() {
  try {
    init_lbr_wrench_command_subscription_();
    this->get_node()->declare_parameter("robot_name", "lbr");
    configure_joint_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR wrench command controller with: %s.", e.what());
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
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to switch to chained mode with: %s.",
                 e.what());
    return false;
  }
  return true;
}

controller_interface::return_type
LBRWrenchCommandController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {
  auto lbr_wrench_command = rt_lbr_wrench_command_ptr_.readFromRT();
  if (!lbr_wrench_command || !(*lbr_wrench_command)) {
    return controller_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    reference_interfaces_[i] = (*lbr_wrench_command)->joint_position[i];
  }
  for (std::size_t i = 0; i < CARTESIAN_DOF; ++i) {
    wrench_command_interfaces_[i].get().set_value((*lbr_wrench_command)->wrench[i]);
  }
  return controller_interface::return_type::OK;
}

// expect just wrench command in this mode....
controller_interface::return_type
LBRWrenchCommandController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {
  if (is_in_chained_mode()) {
    auto wrench_command = rt_wrench_command_ptr_.readFromRT();
    if (!wrench_command || !(*wrench_command)) {
      return controller_interface::return_type::OK;
    }
    wrench_command_interfaces_[0].get().set_value((*wrench_command)->force.x);
    wrench_command_interfaces_[1].get().set_value((*wrench_command)->force.y);
    wrench_command_interfaces_[2].get().set_value((*wrench_command)->force.z);
    wrench_command_interfaces_[3].get().set_value((*wrench_command)->torque.x);
    wrench_command_interfaces_[4].get().set_value((*wrench_command)->torque.y);
    wrench_command_interfaces_[5].get().set_value((*wrench_command)->torque.z);
  }
  // pass joint position and velocity states through to next controller
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_position_states_[i] = joint_position_state_interfaces_[i].get().get_value();
    joint_velocity_states_[i] = joint_velocity_state_interfaces_[i].get().get_value();
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_position_command_interfaces_[i].get().set_value(reference_interfaces_[i]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  estimated_ft_sensor_ptr_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Z,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Z);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  reference_interfaces_.assign(lbr_fri_ros2::N_JNTS, std::numeric_limits<double>::quiet_NaN());
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRWrenchCommandController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_state_interfaces_();
  clear_command_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LBRWrenchCommandController::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.emplace_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      joint_velocity_state_interfaces_.emplace_back(std::ref(state_interface));
    }
  }
  if (!estimated_ft_sensor_ptr_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to assign estimated force torque state interfaces.");
    return false;
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  if (joint_velocity_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint velocity state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_velocity_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  return true;
}

bool LBRWrenchCommandController::reference_command_interfaces_() {
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_command_interfaces_.emplace_back(std::ref(command_interface));
    }
    if (command_interface.get_prefix_name() == HW_IF_WRENCH_PREFIX) {
      wrench_command_interfaces_.emplace_back(std::ref(command_interface));
    }
  }
  if (joint_position_command_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position command interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_command_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  if (wrench_command_interfaces_.size() != CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of wrench command interfaces '%ld' does not equal %d.",
                 wrench_command_interfaces_.size(), CARTESIAN_DOF);
    return false;
  }
  return true;
}

void LBRWrenchCommandController::clear_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  estimated_ft_sensor_ptr_->release_interfaces();
}

void LBRWrenchCommandController::clear_command_interfaces_() {
  joint_position_command_interfaces_.clear();
  wrench_command_interfaces_.clear();
}

void LBRWrenchCommandController::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint names (%ld) does not match the number of joints in the robot (%d).",
        joint_names_.size(), lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void LBRWrenchCommandController::init_lbr_wrench_command_subscription_() {
  lbr_wrench_command_subscription_ptr_ =
      this->get_node()->create_subscription<lbr_fri_idl::msg::LBRWrenchCommand>(
          "command/lbr_wrench_command", 1,
          [this](const lbr_fri_idl::msg::LBRWrenchCommand::SharedPtr msg) {
            rt_lbr_wrench_command_ptr_.writeFromNonRT(msg);
          });
}

void LBRWrenchCommandController::init_wrench_command_subscription_() {
  wrench_command_subscription_ptr_ =
      this->get_node()->create_subscription<geometry_msgs::msg::Wrench>(
          "command/wrench", 1, [this](const geometry_msgs::msg::Wrench::SharedPtr msg) {
            rt_wrench_command_ptr_.writeFromNonRT(msg);
          });
}

void LBRWrenchCommandController::reset_lbr_wrench_command_subscription_() {
  if (wrench_command_subscription_ptr_) {
    wrench_command_subscription_ptr_.reset();
  }
}
void LBRWrenchCommandController::reset_wrench_command_subscription_() {

  if (lbr_wrench_command_subscription_ptr_) {
    lbr_wrench_command_subscription_ptr_.reset();
  }
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRWrenchCommandController,
                       controller_interface::ChainableControllerInterface)
