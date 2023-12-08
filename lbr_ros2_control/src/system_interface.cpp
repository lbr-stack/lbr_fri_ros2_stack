#include "lbr_ros2_control/system_interface.hpp"

namespace lbr_ros2_control {
controller_interface::CallbackReturn
SystemInterface::on_init(const hardware_interface::HardwareInfo &system_info) {
  auto ret = hardware_interface::SystemInterface::on_init(system_info);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to initialize SystemInterface.");
    return ret;
  }

  // parameters_ from config/lbr_system_interface.xacro
  parameters_.port_id = std::stoul(info_.hardware_parameters["port_id"]);
  if (parameters_.port_id < 30200 || parameters_.port_id > 30209) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Expected port_id in [30200, 30209]. Found %d.",
                 parameters_.port_id);
    return controller_interface::CallbackReturn::ERROR;
  }
  info_.hardware_parameters["remote_host"] == "INADDR_ANY"
      ? parameters_.remote_host = NULL
      : parameters_.remote_host = info_.hardware_parameters["remote_host"].c_str();
  parameters_.rt_prio = std::stoul(info_.hardware_parameters["rt_prio"]);
  std::transform(info_.hardware_parameters["open_loop"].begin(),
                 info_.hardware_parameters["open_loop"].end(),
                 info_.hardware_parameters["open_loop"].begin(), ::tolower);
  parameters_.open_loop = info_.hardware_parameters["open_loop"] == "true";
  std::transform(info_.hardware_parameters["pid_antiwindup"].begin(),
                 info_.hardware_parameters["pid_antiwindup"].end(),
                 info_.hardware_parameters["pid_antiwindup"].begin(), ::tolower);
  parameters_.pid_p = std::stod(info_.hardware_parameters["pid_p"]);
  parameters_.pid_i = std::stod(info_.hardware_parameters["pid_i"]);
  parameters_.pid_d = std::stod(info_.hardware_parameters["pid_d"]);
  parameters_.pid_i_max = std::stod(info_.hardware_parameters["pid_i_max"]);
  parameters_.pid_i_min = std::stod(info_.hardware_parameters["pid_i_min"]);
  parameters_.pid_antiwindup = info_.hardware_parameters["pid_antiwindup"] == "true";
  parameters_.command_guard_variant = system_info.hardware_parameters.at("command_guard_variant");
  parameters_.external_torque_cutoff_frequency =
      std::stod(info_.hardware_parameters["external_torque_cutoff_frequency"]);
  parameters_.measured_torque_cutoff_frequency =
      std::stod(info_.hardware_parameters["measured_torque_cutoff_frequency"]);

  // setup driver
  lbr_fri_ros2::PIDParameters pid_parameters;
  lbr_fri_ros2::CommandGuardParameters command_guard_parameters;
  lbr_fri_ros2::StateInterfaceParameters state_interface_parameters;
  pid_parameters.p = parameters_.pid_p;
  pid_parameters.i = parameters_.pid_i;
  pid_parameters.d = parameters_.pid_d;
  pid_parameters.i_max = parameters_.pid_i_max;
  pid_parameters.i_min = parameters_.pid_i_min;
  pid_parameters.antiwindup = parameters_.pid_antiwindup;
  for (std::size_t idx = 0; idx < system_info.joints.size(); ++idx) {
    command_guard_parameters.joint_names[idx] = system_info.joints[idx].name;
    command_guard_parameters.max_position[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_position"));
    command_guard_parameters.min_position[idx] =
        std::stod(system_info.joints[idx].parameters.at("min_position"));
    command_guard_parameters.max_velocity[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_velocity"));
    command_guard_parameters.max_torque[idx] =
        std::stod(system_info.joints[idx].parameters.at("max_torque"));
  }
  state_interface_parameters.external_torque_cutoff_frequency =
      parameters_.external_torque_cutoff_frequency;
  state_interface_parameters.measured_torque_cutoff_frequency =
      parameters_.measured_torque_cutoff_frequency;

  async_client_ptr_ = std::make_shared<lbr_fri_ros2::AsyncClient>(
      pid_parameters, command_guard_parameters, parameters_.command_guard_variant,
      state_interface_parameters, parameters_.open_loop);
  app_ptr_ = std::make_unique<lbr_fri_ros2::App>(async_client_ptr_);

  nan_command_interfaces_();
  nan_state_interfaces_();
  nan_last_hw_states_();

  // setup force-torque estimator
  ft_parameters_.chain_root = info_.sensors[1].parameters.at("chain_root");
  ft_parameters_.chain_tip = info_.sensors[1].parameters.at("chain_tip");
  ft_parameters_.damping = std::stod(info_.sensors[1].parameters.at("damping"));
  ft_parameters_.force_x_th = std::stod(info_.sensors[1].parameters.at("force_x_th"));
  ft_parameters_.force_y_th = std::stod(info_.sensors[1].parameters.at("force_y_th"));
  ft_parameters_.force_z_th = std::stod(info_.sensors[1].parameters.at("force_z_th"));
  ft_parameters_.torque_x_th = std::stod(info_.sensors[1].parameters.at("torque_x_th"));
  ft_parameters_.torque_y_th = std::stod(info_.sensors[1].parameters.at("torque_y_th"));
  ft_parameters_.torque_z_th = std::stod(info_.sensors[1].parameters.at("torque_z_th"));
  ft_estimator_ptr_ = std::make_unique<lbr_fri_ros2::FTEstimator>(
      info_.original_xml, ft_parameters_.chain_root, ft_parameters_.chain_tip,
      lbr_fri_ros2::FTEstimator::cart_array_t{
          ft_parameters_.force_x_th,
          ft_parameters_.force_y_th,
          ft_parameters_.force_z_th,
          ft_parameters_.torque_x_th,
          ft_parameters_.torque_y_th,
          ft_parameters_.torque_z_th,
      });

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

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // state interfaces of type double
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                  &hw_lbr_state_.measured_joint_position[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_JOINT_POSITION,
                                  &hw_lbr_state_.commanded_joint_position[i]);

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                  &hw_lbr_state_.measured_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_TORQUE,
                                  &hw_lbr_state_.commanded_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EXTERNAL_TORQUE,
                                  &hw_lbr_state_.external_torque[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_IPO_JOINT_POSITION,
                                  &hw_lbr_state_.ipo_joint_position[i]);

    // additional velocity state interface
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocity_[i]);
  }

  const auto &auxiliary_sensor = info_.sensors[0];
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SAMPLE_TIME,
                                &hw_lbr_state_.sample_time);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TRACKING_PERFORMANCE,
                                &hw_lbr_state_.tracking_performance);

  // state interfaces that require cast
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SESSION_STATE, &hw_session_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CONNECTION_QUALITY,
                                &hw_connection_quality_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_SAFETY_STATE, &hw_safety_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_OPERATION_MODE, &hw_operation_mode_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_DRIVE_STATE, &hw_drive_state_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CLIENT_COMMAND_MODE,
                                &hw_client_command_mode_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_OVERLAY_TYPE, &hw_overlay_type_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_CONTROL_MODE, &hw_control_mode_);

  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_);
  state_interfaces.emplace_back(auxiliary_sensor.name, HW_IF_TIME_STAMP_NANO_SEC,
                                &hw_time_stamp_nano_sec_);

  // additional force-torque state interface
  const auto &estimated_ft_sensor = info_.sensors[1];
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_X, &hw_ft_[0]);
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_Y, &hw_ft_[1]);
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_FORCE_Z, &hw_ft_[2]);
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_X, &hw_ft_[3]);
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_Y, &hw_ft_[4]);
  state_interfaces.emplace_back(estimated_ft_sensor.name, HW_IF_TORQUE_Z, &hw_ft_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                    &hw_lbr_command_.joint_position[i]);

    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                    &hw_lbr_command_.torque[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type
SystemInterface::prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                                             const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

controller_interface::CallbackReturn SystemInterface::on_activate(const rclcpp_lifecycle::State &) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "AsyncClient not configured.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!app_ptr_->open_udp_socket(parameters_.port_id, parameters_.remote_host)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  app_ptr_->run(parameters_.rt_prio);
  uint8_t attempt = 0;
  uint8_t max_attempts = 10;
  while (!async_client_ptr_->get_state_interface().is_initialized() && rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "Waiting for robot heartbeat [%d/%d]. Port ID: %d.", attempt + 1, max_attempts,
                parameters_.port_id);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (++attempt >= max_attempts) {
      app_ptr_->close_udp_socket(); // hard close as run gets stuck
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to connect to robot on max attempts.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Robot connected.");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Control mode: '%s'.",
              lbr_fri_ros2::EnumMaps::control_mode_map(hw_lbr_state_.control_mode).c_str());
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Sample time: %.3f s / %.1f Hz.",
              async_client_ptr_->get_state_interface().get_state().sample_time,
              1. / async_client_ptr_->get_state_interface().get_state().sample_time);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SystemInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  app_ptr_->stop_run();
  app_ptr_->close_udp_socket();
  return controller_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemInterface::read(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {
  hw_lbr_state_ = async_client_ptr_->get_state_interface().get_state();

  // exit once robot exits COMMANDING_ACTIVE (for safety)
  if (exit_commanding_active_(static_cast<KUKA::FRI::ESessionState>(hw_session_state_),
                              static_cast<KUKA::FRI::ESessionState>(hw_lbr_state_.session_state))) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "LBR left COMMANDING_ACTIVE. Please re-run lbr_bringup.");
    app_ptr_->stop_run();
    app_ptr_->close_udp_socket();
    return hardware_interface::return_type::ERROR;
  }

  // state interfaces that require cast
  hw_session_state_ = static_cast<double>(hw_lbr_state_.session_state);
  hw_connection_quality_ = static_cast<double>(hw_lbr_state_.connection_quality);
  hw_safety_state_ = static_cast<double>(hw_lbr_state_.safety_state);
  hw_operation_mode_ = static_cast<double>(hw_lbr_state_.operation_mode);
  hw_drive_state_ = static_cast<double>(hw_lbr_state_.drive_state);
  hw_client_command_mode_ = static_cast<double>(hw_lbr_state_.client_command_mode);
  hw_overlay_type_ = static_cast<double>(hw_lbr_state_.overlay_type);
  hw_control_mode_ = static_cast<double>(hw_lbr_state_.control_mode);
  hw_time_stamp_sec_ = static_cast<double>(hw_lbr_state_.time_stamp_sec);
  hw_time_stamp_nano_sec_ = static_cast<double>(hw_lbr_state_.time_stamp_nano_sec);

  // additional velocity state interface
  compute_hw_velocity_();
  update_last_hw_states_();

  // additional force-torque state interface
  ft_estimator_ptr_->compute(hw_lbr_state_.measured_joint_position, hw_lbr_state_.external_torque,
                             hw_ft_, ft_parameters_.damping);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemInterface::write(const rclcpp::Time & /*time*/,
                                                       const rclcpp::Duration & /*period*/) {
  if (hw_session_state_ != KUKA::FRI::COMMANDING_ACTIVE) {
    return hardware_interface::return_type::OK;
  }
  if (hw_client_command_mode_ == KUKA::FRI::EClientCommandMode::POSITION) {
    if (std::any_of(hw_lbr_command_.joint_position.cbegin(), hw_lbr_command_.joint_position.cend(),
                    [](const double &v) { return std::isnan(v); })) {
      return hardware_interface::return_type::OK;
    }
    async_client_ptr_->get_command_interface().set_command_target(hw_lbr_command_);
    return hardware_interface::return_type::OK;
  }
  if (hw_client_command_mode_ == KUKA::FRI::EClientCommandMode::TORQUE) {
    if (std::any_of(hw_lbr_command_.joint_position.cbegin(), hw_lbr_command_.joint_position.cend(),
                    [](const double &v) { return std::isnan(v); }) ||
        std::any_of(hw_lbr_command_.torque.cbegin(), hw_lbr_command_.torque.cend(),
                    [](const double &v) { return std::isnan(v); })) {
      return hardware_interface::return_type::OK;
    }
    async_client_ptr_->get_command_interface().set_command_target(hw_lbr_command_);
    return hardware_interface::return_type::OK;
  }
  if (hw_client_command_mode_ == KUKA::FRI::EClientCommandMode::WRENCH) {
    throw std::runtime_error("Wrench command mode currently not implemented.");
  }
  return hardware_interface::return_type::ERROR;
}

void SystemInterface::nan_command_interfaces_() {
  hw_lbr_command_.joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_command_.torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_command_.wrench.fill(std::numeric_limits<double>::quiet_NaN());
}

void SystemInterface::nan_state_interfaces_() {
  // state interfaces of type double
  hw_lbr_state_.measured_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.commanded_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.measured_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.commanded_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.external_torque.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.ipo_joint_position.fill(std::numeric_limits<double>::quiet_NaN());
  hw_lbr_state_.sample_time = std::numeric_limits<double>::quiet_NaN();
  hw_lbr_state_.tracking_performance = std::numeric_limits<double>::quiet_NaN();

  // state interfaces that require cast
  hw_session_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_connection_quality_ = std::numeric_limits<double>::quiet_NaN();
  hw_safety_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_operation_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_drive_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_client_command_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_overlay_type_ = std::numeric_limits<double>::quiet_NaN();
  hw_control_mode_ = std::numeric_limits<double>::quiet_NaN();
  hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();

  // additional velocity state interface
  hw_velocity_.fill(std::numeric_limits<double>::quiet_NaN());

  // additional force-torque state interface
  hw_ft_.fill(std::numeric_limits<double>::quiet_NaN());
}

bool SystemInterface::verify_number_of_joints_() {
  if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Expected %d joints in URDF. Found %ld.",
                 KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
    return false;
  }
  return true;
}

bool SystemInterface::verify_joint_command_interfaces_() {
  // check command interfaces
  for (auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE) {
      RCLCPP_ERROR(
          rclcpp::get_logger(LOGGER_NAME),
          "Joint %s received invalid number of command interfaces. Received %ld, expected %d.",
          joint.name.c_str(), joint.command_interfaces.size(), LBR_FRI_COMMAND_INTERFACE_SIZE);
      return false;
    }
    for (auto &ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Joint %s received invalid command interface: %s. Expected %s or %s.",
                     joint.name.c_str(), ci.name.c_str(), hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_EFFORT);
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
      RCLCPP_ERROR(
          rclcpp::get_logger(LOGGER_NAME),
          "Joint %s received invalid number of state interfaces. Received %ld, expected %d.",
          joint.name.c_str(), joint.state_interfaces.size(), LBR_FRI_STATE_INTERFACE_SIZE);
      return false;
    }
    for (auto &si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != HW_IF_COMMANDED_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_EFFORT && si.name != HW_IF_COMMANDED_TORQUE &&
          si.name != HW_IF_EXTERNAL_TORQUE && si.name != HW_IF_IPO_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR(
            rclcpp::get_logger(LOGGER_NAME),
            "Joint %s received invalid state interface: %s. Expected %s, %s, %s, %s, %s, %s or %s.",
            joint.name.c_str(), si.name.c_str(), hardware_interface::HW_IF_POSITION,
            HW_IF_COMMANDED_JOINT_POSITION, hardware_interface::HW_IF_EFFORT,
            HW_IF_COMMANDED_TORQUE, HW_IF_EXTERNAL_TORQUE, HW_IF_IPO_JOINT_POSITION,
            hardware_interface::HW_IF_VELOCITY);
        return false;
      }
    }
  }
  return true;
}

bool SystemInterface::verify_sensors_() {
  // check lbr specific state interfaces
  if (info_.sensors.size() != LBR_FRI_SENSORS) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Expected %d sensor, got %ld", LBR_FRI_SENSORS,
                 info_.sensors.size());
    return false;
  }
  if (!verify_auxiliary_sensor_()) {
    return false;
  }
  if (!verify_estimated_ft_sensor_()) {
    return false;
  }
  return true;
}

bool SystemInterface::verify_auxiliary_sensor_() {
  // check all interfaces are defined in config/lbr_system_interface.xacro
  const auto &auxiliary_sensor = info_.sensors[0];
  if (auxiliary_sensor.state_interfaces.size() != AUXILIARY_SENSOR_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Sensor %s received invalid state interface. Received %ld, expected %d. ",
                 auxiliary_sensor.name.c_str(), auxiliary_sensor.state_interfaces.size(),
                 AUXILIARY_SENSOR_SIZE);
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
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                   "Sensor %s received invalid state interface %s.", auxiliary_sensor.name.c_str(),
                   si.name.c_str());
      return false;
    }
  }
  return true;
}

bool SystemInterface::verify_estimated_ft_sensor_() {
  const auto &estimated_ft_sensor = info_.sensors[1];
  if (estimated_ft_sensor.state_interfaces.size() != ESTIMATED_FT_SENSOR_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Sensor %s received invalid state interface. Received %ld, expected %d. ",
                 estimated_ft_sensor.name.c_str(), estimated_ft_sensor.state_interfaces.size(),
                 ESTIMATED_FT_SENSOR_SIZE);
    return false;
  }
  // check only valid interfaces are defined
  for (const auto &si : estimated_ft_sensor.state_interfaces) {
    if (si.name != HW_IF_FORCE_X && si.name != HW_IF_FORCE_Y && si.name != HW_IF_FORCE_Z &&
        si.name != HW_IF_TORQUE_X && si.name != HW_IF_TORQUE_Y && si.name != HW_IF_TORQUE_Z) {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                   "Sensor %s received invalid state interface %s.",
                   estimated_ft_sensor.name.c_str(), si.name.c_str());
      return false;
    }
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

void SystemInterface::nan_last_hw_states_() {
  last_hw_measured_joint_position_.fill(std::numeric_limits<double>::quiet_NaN());
  last_hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  last_hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();
}

void SystemInterface::update_last_hw_states_() {
  last_hw_measured_joint_position_ = hw_lbr_state_.measured_joint_position;
  last_hw_time_stamp_sec_ = hw_time_stamp_sec_;
  last_hw_time_stamp_nano_sec_ = hw_time_stamp_nano_sec_;
}

void SystemInterface::compute_hw_velocity_() {
  // state uninitialized
  if (std::isnan(last_hw_time_stamp_nano_sec_) || std::isnan(last_hw_measured_joint_position_[0])) {
    return;
  }

  // state wasn't updated
  if (last_hw_time_stamp_sec_ == hw_time_stamp_sec_ &&
      last_hw_time_stamp_nano_sec_ == hw_time_stamp_nano_sec_) {
    return;
  }

  double dt = time_stamps_to_sec_(hw_time_stamp_sec_, hw_time_stamp_nano_sec_) -
              time_stamps_to_sec_(last_hw_time_stamp_sec_, last_hw_time_stamp_nano_sec_);
  std::size_t i = 0;
  std::for_each(hw_velocity_.begin(), hw_velocity_.end(), [&](double &v) {
    v = (hw_lbr_state_.measured_joint_position[i] - last_hw_measured_joint_position_[i]) / dt;
    ++i;
  });
}

} // end of namespace lbr_ros2_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::SystemInterface, hardware_interface::SystemInterface)
