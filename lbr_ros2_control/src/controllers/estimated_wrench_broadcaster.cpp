#include "lbr_ros2_control/controllers/estimated_wrench_broadcaster.hpp"

namespace lbr_ros2_control {
EstimatedWrenchBroadcaster::EstimatedWrenchBroadcaster() {}

controller_interface::InterfaceConfiguration
EstimatedWrenchBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
EstimatedWrenchBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position and external torque interfaces
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + HW_IF_EXTERNAL_TORQUE);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn EstimatedWrenchBroadcaster::on_init() {
  try {
    get_node()->declare_parameter("robot_name", "lbr");
    get_node()->declare_parameter("wrench_estimator_parameters.chain_root", "lbr_link_0");
    get_node()->declare_parameter("wrench_estimator_parameters.chain_tip", "lbr_link_ee");
    get_node()->declare_parameter("wrench_estimator_parameters.damping", 0.2);
    get_node()->declare_parameter("wrench_estimator_parameters.force_x_th", 2.0);
    get_node()->declare_parameter("wrench_estimator_parameters.force_y_th", 2.0);
    get_node()->declare_parameter("wrench_estimator_parameters.force_z_th", 2.0);
    get_node()->declare_parameter("wrench_estimator_parameters.torque_x_th", 0.5);
    get_node()->declare_parameter("wrench_estimator_parameters.torque_y_th", 0.5);
    get_node()->declare_parameter("wrench_estimator_parameters.torque_z_th", 0.5);
    configure_joint_names_();
    configure_parameters_();
    wrench_estimator_ptr_ = std::make_unique<lbr_fri_ros2::WrenchEstimator>(
        get_robot_description(), wrench_estimator_parameters_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to initialize estimated wrench broadcaster with: "
                            << e.what() << "." << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EstimatedWrenchBroadcaster::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_X, &wrench_[0]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_Y, &wrench_[1]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_FORCE_Z, &wrench_[2]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_X, &wrench_[3]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_Y, &wrench_[4]);
  state_interfaces.emplace_back(std::string(get_node()->get_name()), HW_IF_TORQUE_Z, &wrench_[5]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EstimatedWrenchBroadcaster::on_export_reference_interfaces() {
  return {};
}

bool EstimatedWrenchBroadcaster::on_set_chained_mode(bool chained_mode) {
  RCLCPP_INFO(get_node()->get_logger(), "EstimatedWrenchBroadcaster::on_set_chained_mode");
  if (chained_mode) {
    // delete publisher
  } else {
    // create publisher
  }
  return true;
} // in chained mode expose force-torque state interface, else publish...

controller_interface::return_type
EstimatedWrenchBroadcaster::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {
  //   RCLCPP_INFO(get_node()->get_logger(),
  //               "EstimatedWrenchBroadcaster::update_reference_from_subscribers");
  return controller_interface::return_type::OK;
} // do nothing...

controller_interface::return_type
EstimatedWrenchBroadcaster::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {
  // get joint positions
  if (!read_state_interfaces_()) {
    return controller_interface::return_type::OK;
  }

  //   RCLCPP_INFO(get_node()->get_logger(),
  //   "EstimatedWrenchBroadcaster::update_and_write_commands");
  wrench_estimator_ptr_->set_q(joint_positions_);
  wrench_estimator_ptr_->set_tau_ext(external_torques_);
  wrench_estimator_ptr_->compute();
  wrench_estimator_ptr_->get_f_ext_tf(wrench_);
  return controller_interface::return_type::OK;
} // do nothing...

controller_interface::CallbackReturn
EstimatedWrenchBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EstimatedWrenchBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!assign_state_interfaces_()) {
    release_state_interfaces_();
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EstimatedWrenchBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  release_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool EstimatedWrenchBroadcaster::assign_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.push_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_EXTERNAL_TORQUE) {
      external_torque_state_interfaces_.push_back(std::ref(state_interface));
    }
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint position state interfaces '"
                            << joint_position_state_interfaces_.size()
                            << "' does not match the number of joints "
                               "in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (external_torque_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Number of external torque interfaces '"
                                                      << external_torque_state_interfaces_.size()
                                                      << "' does not match the number of joints "
                                                         "in the robot '"
                                                      << lbr_fri_ros2::N_JNTS << "'."
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void EstimatedWrenchBroadcaster::release_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  external_torque_state_interfaces_.clear();
}

void EstimatedWrenchBroadcaster::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Number of joint names '" << joint_names_.size()
                            << "' does not match the number of joints in the robot '"
                            << lbr_fri_ros2::N_JNTS << "'." << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void EstimatedWrenchBroadcaster::configure_parameters_() {
  wrench_estimator_parameters_.chain_root =
      get_node()->get_parameter("wrench_estimator_parameters.chain_root").as_string();
  wrench_estimator_parameters_.chain_tip =
      get_node()->get_parameter("wrench_estimator_parameters.chain_tip").as_string();
  wrench_estimator_parameters_.damping =
      get_node()->get_parameter("wrench_estimator_parameters.damping").as_double();
  wrench_estimator_parameters_.force_x_th =
      get_node()->get_parameter("wrench_estimator_parameters.force_x_th").as_double();
  wrench_estimator_parameters_.force_y_th =
      get_node()->get_parameter("wrench_estimator_parameters.force_y_th").as_double();
  wrench_estimator_parameters_.force_z_th =
      get_node()->get_parameter("wrench_estimator_parameters.force_z_th").as_double();
  wrench_estimator_parameters_.torque_x_th =
      get_node()->get_parameter("wrench_estimator_parameters.torque_x_th").as_double();
  wrench_estimator_parameters_.torque_y_th =
      get_node()->get_parameter("wrench_estimator_parameters.torque_y_th").as_double();
  wrench_estimator_parameters_.torque_z_th =
      get_node()->get_parameter("wrench_estimator_parameters.torque_z_th").as_double();
  if (!wrench_estimator_parameters_.valid()) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Invalid wrench estimator parameters."
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    throw std::runtime_error("Invalid wrench estimator parameters.");
  }
}

bool EstimatedWrenchBroadcaster::read_state_interfaces_() {
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    auto q_i = joint_position_state_interfaces_[i].get().get_optional();
    if (!q_i.has_value()) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::WARNING
                                                       << "Failed to get joint position for joint '"
                                                       << joint_names_[i] << "'."
                                                       << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    joint_positions_[i] = *q_i;

    auto tau_ext_i = external_torque_state_interfaces_[i].get().get_optional();
    if (!tau_ext_i.has_value()) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(),
                         lbr_fri_ros2::ColorScheme::WARNING
                             << "Failed to get external torque for joint '" << joint_names_[i]
                             << "'." << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    external_torques_[i] = *tau_ext_i;
  }
  return true;
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::EstimatedWrenchBroadcaster,
                       controller_interface::ChainableControllerInterface)
