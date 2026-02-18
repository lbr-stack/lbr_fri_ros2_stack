#include "lbr_ros2_control/controllers/lbr_torque_command_controller.hpp"

namespace lbr_ros2_control {
LBRTorqueCommandController::LBRTorqueCommandController()
    : rt_lbr_torque_command_ptr_(nullptr), lbr_torque_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRTorqueCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRTorqueCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LBRTorqueCommandController::on_init() {
  try {
    lbr_torque_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::LBRTorqueCommand>(
            "command/lbr_torque_command", 1,
            [this](const lbr_fri_idl::msg::LBRTorqueCommand::SharedPtr msg) {
              rt_lbr_torque_command_ptr_.writeFromNonRT(msg);
            });
    this->get_node()->declare_parameter("robot_name", "lbr");
    configure_joint_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to initialize LBR torque command controller with: "
                            << e.what() << "." << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRTorqueCommandController::update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {
  auto lbr_torque_command = rt_lbr_torque_command_ptr_.readFromRT();
  if (!lbr_torque_command || !(*lbr_torque_command)) {
    return controller_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    if (!joint_position_command_interfaces_[i].get().set_value(
            (*lbr_torque_command)->joint_position[i])) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Failed to set joint position for joint '" << joint_names_[i]
                              << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    }
    if (!torque_command_interfaces_[i].get().set_value((*lbr_torque_command)->torque[i])) {
      RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Failed to set joint troque for joint '" << joint_names_[i] << "'."
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBRTorqueCommandController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRTorqueCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!assign_command_interfaces_()) {
    release_command_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRTorqueCommandController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_command_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LBRTorqueCommandController::assign_command_interfaces_() {
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_command_interfaces_.push_back(std::ref(command_interface));
    }
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      torque_command_interfaces_.push_back(std::ref(command_interface));
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
  if (torque_command_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR << "Number of torque command interfaces '"
                                                         << torque_command_interfaces_.size()
                                                         << "' does not match the number of joints "
                                                            "in the robot '"
                                                         << lbr_fri_ros2::N_JNTS << "'."
                                                         << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void LBRTorqueCommandController::release_command_interfaces_() {
  joint_position_command_interfaces_.clear();
  torque_command_interfaces_.clear();
}

void LBRTorqueCommandController::configure_joint_names_() {
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
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRTorqueCommandController,
                       controller_interface::ControllerInterface)
