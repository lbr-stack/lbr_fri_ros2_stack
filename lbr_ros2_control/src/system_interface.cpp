#include "lbr_ros2_control/system_interface.hpp"

namespace lbr_ros2_control {
controller_interface::CallbackReturn
SystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
  auto ret = hardware_interface::SystemInterface::on_init(params); // parses params to info_
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Failed to initialize SystemInterface"
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return ret;
  }

  // parameters_ from lbr_system_interface.xacro (default configurations located in
  // lbr_description/ros2_control/lbr_system_interface.xacro)
  if (!parse_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // setup driver
  lbr_fri_ros2::CommandGuardParameters command_guard_parameters;
  lbr_fri_ros2::StateGuardParameters state_guard_parameters;
  lbr_fri_ros2::StateInterfaceParameters state_interface_parameters;
  for (std::size_t idx = 0; idx < info_.joints.size(); ++idx) {
    command_guard_parameters.joint_names[idx] = info_.joints[idx].name;
    command_guard_parameters.max_positions[idx] =
        info_.limits.at(info_.joints[idx].name).max_position;
    command_guard_parameters.min_positions[idx] =
        info_.limits.at(info_.joints[idx].name).min_position;
    command_guard_parameters.max_velocities[idx] =
        info_.limits.at(info_.joints[idx].name).max_velocity;
    command_guard_parameters.max_torques[idx] = info_.limits.at(info_.joints[idx].name).max_effort;

    // currently, only check external torque limits on enter commanding active with fixed limit, see
    // https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/271#issuecomment-2780642918
    state_guard_parameters.joint_names[idx] = info_.joints[idx].name;
    state_guard_parameters.max_external_torque[idx] = parameters_.state_guard_external_torque_limit;
  }
  state_guard_parameters.external_torque_safety_check =
      parameters_.state_guard_external_torque_safety_check;
  state_interface_parameters.external_torque_tau = parameters_.external_torque_tau;
  state_interface_parameters.measured_torque_tau = parameters_.measured_torque_tau;

  try {
    async_client_ptr_ = std::make_shared<lbr_fri_ros2::AsyncClient>(
        parameters_.client_command_mode, parameters_.joint_position_tau, command_guard_parameters,
        parameters_.command_guard_variant, state_guard_parameters, state_interface_parameters,
        parameters_.open_loop);
    app_ptr_ = std::make_unique<lbr_fri_ros2::App>(async_client_ptr_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to instantiate AsyncClient or App with: " << e.what()
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }

  // perform verifications
  if (!verify_number_of_joints_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_joint_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_joint_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_sensors_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!verify_gpios_()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SystemInterface::prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                                             const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

controller_interface::CallbackReturn
SystemInterface::on_configure(const rclcpp_lifecycle::State &) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "AsyncClient not configured"
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!app_ptr_->open_udp_socket(parameters_.port_id, parameters_.remote_host)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SystemInterface::on_activate(const rclcpp_lifecycle::State &) {
  // populate the interface handles
  command_if_handles_.populate(*this);
  state_if_handles_.populate(*this);

  // nan all command and state interfaces (this might be removed in the future with ros2_control
  // handling them now)
  nan_command_interfaces_();
  nan_state_interfaces_();
  nan_last_states_();

  // run the FRI app
  app_ptr_->run_async(parameters_.rt_prio);
  int attempt = 0;
  while (!async_client_ptr_->get_state_interface()->is_initialized()) {
    RCLCPP_INFO_STREAM(
        get_node()->get_logger(),
        "Awaiting robot heartbeat. Attempt "
            << ++attempt << ", remote_host '" << lbr_fri_ros2::ColorScheme::OKBLUE
            << lbr_fri_ros2::ColorScheme::BOLD
            << (parameters_.remote_host == NULL ? "INADDR_ANY" : parameters_.remote_host)
            << lbr_fri_ros2::ColorScheme::ENDC << "', port_id '"
            << lbr_fri_ros2::ColorScheme::OKBLUE << lbr_fri_ros2::ColorScheme::BOLD
            << parameters_.port_id << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    if (!rclcpp::ok()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::OKGREEN
                                                   << "Robot connected"
                                                   << lbr_fri_ros2::ColorScheme::ENDC);
  auto state = async_client_ptr_->get_state_interface()->get_state();
  RCLCPP_INFO(get_node()->get_logger(), "Sample time %.3f s / %.1f Hz", state.sample_time,
              1. / state.sample_time);
  while (!(state.session_state >= KUKA::FRI::ESessionState::COMMANDING_WAIT)) {
    state = async_client_ptr_->get_state_interface()->get_state();
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Awaiting '"
                           << lbr_fri_ros2::ColorScheme::BOLD << lbr_fri_ros2::ColorScheme::OKBLUE
                           << lbr_fri_ros2::EnumMaps::session_state_map(
                                  KUKA::FRI::ESessionState::COMMANDING_WAIT)
                           << lbr_fri_ros2::ColorScheme::ENDC << "' state. Current state '"
                           << lbr_fri_ros2::ColorScheme::BOLD << lbr_fri_ros2::ColorScheme::OKBLUE
                           << lbr_fri_ros2::EnumMaps::session_state_map(state.session_state)
                           << lbr_fri_ros2::ColorScheme::ENDC << "'.");
    if (state.session_state == KUKA::FRI::ESessionState::IDLE) {
      RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          lbr_fri_ros2::ColorScheme::ERROR
              << "Robot in '"
              << lbr_fri_ros2::EnumMaps::session_state_map(KUKA::FRI::ESessionState::IDLE)
              << "' state. Please restart the FRI server." << lbr_fri_ros2::ColorScheme::ENDC);
      app_ptr_->close_udp_socket(); // no need to request stop since already stopped when IDLE
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!rclcpp::ok()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  // initialize the previous session state
  previous_session_state_ = static_cast<KUKA::FRI::ESessionState>(state.session_state);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SystemInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  app_ptr_->request_stop();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SystemInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  app_ptr_->close_udp_socket();
  return controller_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemInterface::read(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration &period) {
  if (!async_client_ptr_->get_state_interface()->is_initialized()) {
    return hardware_interface::return_type::OK;
  }

  lbr_state_ = async_client_ptr_->get_state_interface()->get_state();

  if (period.seconds() - lbr_state_.sample_time * 0.2 > lbr_state_.sample_time) {
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 500 /*ms*/,
                                lbr_fri_ros2::ColorScheme::WARNING
                                    << "Increase update_rate parameter for controller_manager to "
                                    << std::to_string(static_cast<int>(1. / lbr_state_.sample_time))
                                    << " Hz or more" << lbr_fri_ros2::ColorScheme::ENDC);
  }

  // exit once robot exits COMMANDING_ACTIVE (for safety)
  auto current_session_state = static_cast<KUKA::FRI::ESessionState>(lbr_state_.session_state);
  if (exit_commanding_active_(previous_session_state_, current_session_state)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "LBR left COMMANDING_ACTIVE. Please re-run lbr_bringup"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    app_ptr_->request_stop();
    app_ptr_->close_udp_socket();
    return hardware_interface::return_type::ERROR;
  }
  previous_session_state_ = current_session_state;

  // compute velocity
  compute_velocity_();

  // set the joint state interfaces
  state_if_handles_.push(lbr_state_, velocity_);

  update_last_states_();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemInterface::write(const rclcpp::Time & /*time*/,
                                                       const rclcpp::Duration & /*period*/) {
  if (lbr_state_.session_state != KUKA::FRI::COMMANDING_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  // populate command message
  command_if_handles_.pull(lbr_command_);

  // forward message to fri
  async_client_ptr_->get_command_interface()->buffer_command_target(lbr_command_);
  return hardware_interface::return_type::OK;
}

bool SystemInterface::parse_parameters_() {
  try {
    parameters_.fri_client_sdk_major_version =
        std::stoul(info_.hardware_parameters.at("fri_client_sdk_major_version"));
    parameters_.fri_client_sdk_minor_version =
        std::stoul(info_.hardware_parameters.at("fri_client_sdk_minor_version"));
    if (parameters_.fri_client_sdk_major_version != FRI_CLIENT_VERSION_MAJOR) {
      RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          lbr_fri_ros2::ColorScheme::ERROR
              << "Expected FRI client SDK version '" << FRI_CLIENT_VERSION_MAJOR << "', got '"
              << std::to_string(parameters_.fri_client_sdk_major_version)
              << "'. Update lbr_system_config.yaml or compile against correct FRI version."
              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    std::string client_command_mode = info_.hardware_parameters.at("client_command_mode");
    if (client_command_mode == "position") {
#if FRI_CLIENT_VERSION_MAJOR == 1
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::POSITION;
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::JOINT_POSITION;
#endif
    } else if (client_command_mode == "torque") {
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::TORQUE;
    } else if (client_command_mode == "wrench") {
      parameters_.client_command_mode = KUKA::FRI::EClientCommandMode::WRENCH;
    } else {
      RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          lbr_fri_ros2::ColorScheme::ERROR
              << "Expected client_command_mode 'position', 'torque' or 'wrench', got '"
              << lbr_fri_ros2::ColorScheme::BOLD << client_command_mode << "'"
              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    parameters_.port_id = std::stoul(info_.hardware_parameters.at("port_id"));
    if (parameters_.port_id < 30200 || parameters_.port_id > 30209) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Expected port_id in [30200, 30209], got '"
                              << lbr_fri_ros2::ColorScheme::BOLD << parameters_.port_id << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    info_.hardware_parameters.at("remote_host") == "INADDR_ANY"
        ? parameters_.remote_host = NULL
        : parameters_.remote_host = info_.hardware_parameters.at("remote_host").c_str();
    parameters_.rt_prio = std::stoul(info_.hardware_parameters.at("rt_prio"));
    parameters_.joint_position_tau = std::stod(info_.hardware_parameters.at("joint_position_tau"));
    parameters_.command_guard_variant = info_.hardware_parameters.at("command_guard_variant");
    std::transform(info_.hardware_parameters.at("state_guard_external_torque_safety_check").begin(),
                   info_.hardware_parameters.at("state_guard_external_torque_safety_check").end(),
                   info_.hardware_parameters.at("state_guard_external_torque_safety_check").begin(),
                   ::tolower); // convert to lower case
    parameters_.state_guard_external_torque_safety_check =
        info_.hardware_parameters.at("state_guard_external_torque_safety_check") == "true";
    parameters_.state_guard_external_torque_limit =
        std::stod(info_.hardware_parameters.at("state_guard_external_torque_limit"));
    parameters_.external_torque_tau =
        std::stod(info_.hardware_parameters.at("external_torque_tau"));
    parameters_.measured_torque_tau =
        std::stod(info_.hardware_parameters.at("measured_torque_tau"));
    std::transform(info_.hardware_parameters.at("open_loop").begin(),
                   info_.hardware_parameters.at("open_loop").end(),
                   info_.hardware_parameters.at("open_loop").begin(),
                   ::tolower); // convert to lower case
    parameters_.open_loop = info_.hardware_parameters.at("open_loop") == "true";
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Failed to parse hardware parameters with: " << e.what()
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

void SystemInterface::nan_command_interfaces_() { command_if_handles_.nan_interfaces(); }

void SystemInterface::nan_state_interfaces_() { state_if_handles_.nan_interfaces(); }

bool SystemInterface::verify_number_of_joints_() {
  if (info_.joints.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Expected '" << lbr_fri_ros2::N_JNTS << "' joints in URDF, got '"
                            << info_.joints.size() << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

bool SystemInterface::verify_joint_command_interfaces_() {
  // check command interfaces
  for (auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Joint '" << joint.name.c_str()
                              << "' received invalid number of command interfaces. Received '"
                              << joint.command_interfaces.size() << "', expected "
                              << static_cast<int>(LBR_FRI_COMMAND_INTERFACE_SIZE)
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    for (auto &ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                            lbr_fri_ros2::ColorScheme::ERROR
                                << "Joint '" << joint.name.c_str()
                                << "' received invalid command interface '" << ci.name.c_str()
                                << "'. Expected '" << hardware_interface::HW_IF_POSITION << "' or '"
                                << hardware_interface::HW_IF_EFFORT << "'"
                                << lbr_fri_ros2::ColorScheme::ENDC);
        return false;
      }
    }
  }
  return true;
}

bool SystemInterface::verify_joint_state_interfaces_() {
  // check state interfaces
  for (auto &joint : info_.joints) {
    if (joint.state_interfaces.size() != LBR_FRI_STATE_INTERFACE_SIZE) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Joint '" << joint.name.c_str()
                              << "' received invalid number of state interfaces. Received '"
                              << joint.state_interfaces.size() << "', expected '"
                              << static_cast<int>(LBR_FRI_STATE_INTERFACE_SIZE) << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
    for (auto &si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != HW_IF_COMMANDED_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_EFFORT && si.name != HW_IF_COMMANDED_TORQUE &&
          si.name != HW_IF_EXTERNAL_TORQUE && si.name != HW_IF_IPO_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR_STREAM(
            get_node()->get_logger(),
            lbr_fri_ros2::ColorScheme::ERROR
                << "Joint '" << joint.name.c_str() << "' received invalid state interface '"
                << si.name.c_str() << "'. Expected one of '" << hardware_interface::HW_IF_POSITION
                << "', '" << HW_IF_COMMANDED_JOINT_POSITION << "', '"
                << hardware_interface::HW_IF_EFFORT << "', '" << HW_IF_COMMANDED_TORQUE << "', '"
                << HW_IF_EXTERNAL_TORQUE << "', '" << HW_IF_IPO_JOINT_POSITION << "' or '"
                << hardware_interface::HW_IF_VELOCITY << "'" << lbr_fri_ros2::ColorScheme::ENDC);
        return false;
      }
    }
  }
  return true;
}

bool SystemInterface::verify_sensors_() {
  // check lbr specific state interfaces
  if (info_.sensors.size() != LBR_FRI_SENSORS) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Expected '"
                                                      << static_cast<int>(LBR_FRI_SENSORS)
                                                      << "' sensors, got '" << info_.sensors.size()
                                                      << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (!verify_auxiliary_sensor_()) {
    return false;
  }
  return true;
}

bool SystemInterface::verify_auxiliary_sensor_() {
  // check all interfaces are defined in lbr_system_interface.xacro (located in
  // lbr_description/ros2_control/lbr_system_interface.xacro)
  const auto &auxiliary_sensor = info_.sensors[0];
  if (auxiliary_sensor.name != HW_IF_AUXILIARY_PREFIX) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Sensor '" << auxiliary_sensor.name.c_str()
                                                      << "' received invalid name. Expected '"
                                                      << HW_IF_AUXILIARY_PREFIX << "'"
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (auxiliary_sensor.state_interfaces.size() != AUXILIARY_SENSOR_SIZE) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "Sensor '" << auxiliary_sensor.name.c_str()
                            << "' received invalid number of state interfaces." << " Received '"
                            << auxiliary_sensor.state_interfaces.size() << "', expected '"
                            << static_cast<int>(AUXILIARY_SENSOR_SIZE) << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  // check only valid interfaces are defined
  for (const auto &si : auxiliary_sensor.state_interfaces) {
    if (si.name != HW_IF_SAMPLE_TIME && si.name != HW_IF_SESSION_STATE &&
        si.name != HW_IF_CONNECTION_QUALITY && si.name != HW_IF_SAFETY_STATE &&
        si.name != HW_IF_OPERATION_MODE && si.name != HW_IF_DRIVE_STATE &&
        si.name != HW_IF_CLIENT_COMMAND_MODE && si.name != HW_IF_OVERLAY_TYPE &&
        si.name != HW_IF_CONTROL_MODE && si.name != HW_IF_TIME_STAMP_SEC &&
        si.name != HW_IF_TIME_STAMP_NANO_SEC && si.name != HW_IF_COMMANDED_JOINT_POSITION &&
        si.name != HW_IF_COMMANDED_TORQUE && si.name != HW_IF_EXTERNAL_TORQUE &&
        si.name != HW_IF_IPO_JOINT_POSITION && si.name != HW_IF_TRACKING_PERFORMANCE) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          lbr_fri_ros2::ColorScheme::ERROR
                              << "Sensor '" << auxiliary_sensor.name.c_str()
                              << "' received invalid state interface '" << si.name.c_str() << "'"
                              << lbr_fri_ros2::ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool SystemInterface::verify_gpios_() {
  if (info_.gpios.size() != GPIO_SIZE) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "Expected '" << static_cast<int>(GPIO_SIZE)
                                                      << "' GPIOs, got '" << info_.gpios.size()
                                                      << "'" << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (info_.gpios[0].name != HW_IF_WRENCH_PREFIX) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), lbr_fri_ros2::ColorScheme::ERROR
                                                      << "GPIO '" << info_.gpios[0].name.c_str()
                                                      << "' received invalid name. Expected '"
                                                      << HW_IF_WRENCH_PREFIX << "'"
                                                      << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  if (info_.gpios[0].command_interfaces.size() != lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        lbr_fri_ros2::ColorScheme::ERROR
                            << "GPIO '" << info_.gpios[0].name.c_str()
                            << "' received invalid number of command interfaces. Received '"
                            << info_.gpios[0].command_interfaces.size() << "', expected '"
                            << lbr_fri_ros2::CARTESIAN_DOF << "'"
                            << lbr_fri_ros2::ColorScheme::ENDC);
    return false;
  }
  return true;
}

bool SystemInterface::exit_commanding_active_(
    const KUKA::FRI::ESessionState &previous_session_state,
    const KUKA::FRI::ESessionState &session_state) {
  if (previous_session_state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE &&
      previous_session_state != session_state) {
    return true;
  }
  return false;
}

double SystemInterface::time_stamps_to_sec_(const double &sec, const double &nano_sec) const {
  return sec + nano_sec / 1.e9;
}

void SystemInterface::nan_last_states_() {
  last_measured_joint_position_.fill(std::numeric_limits<double>::quiet_NaN());
  last_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  last_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();
}

void SystemInterface::update_last_states_() {
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    last_measured_joint_position_[i] = lbr_state_.measured_joint_position[i];
  }
  last_time_stamp_sec_ = lbr_state_.time_stamp_sec;
  last_time_stamp_nano_sec_ = lbr_state_.time_stamp_nano_sec;
}

void SystemInterface::compute_velocity_() {
  // state uninitialized
  if (std::isnan(last_time_stamp_nano_sec_) || std::isnan(last_measured_joint_position_[0])) {
    return;
  }

  const double time_stamp_sec = lbr_state_.time_stamp_sec;
  const double time_stamp_nano_sec = lbr_state_.time_stamp_nano_sec;

  // state wasn't updated
  if (last_time_stamp_sec_ == time_stamp_sec && last_time_stamp_nano_sec_ == time_stamp_nano_sec) {
    return;
  }

  double dt = time_stamps_to_sec_(time_stamp_sec, time_stamp_nano_sec) -
              time_stamps_to_sec_(last_time_stamp_sec_, last_time_stamp_nano_sec_);
  if (dt <= 0) {
    return;
  }
  for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    velocity_[i] = (lbr_state_.measured_joint_position[i] - last_measured_joint_position_[i]) / dt;
  }
}

} // namespace lbr_ros2_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::SystemInterface, hardware_interface::SystemInterface)
