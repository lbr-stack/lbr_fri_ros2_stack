#include "lbr_hardware_interface/lbr_hardware_interface.hpp"

namespace lbr_hardware_interface {
hardware_interface::return_type
LBRHardwareInterface::configure(const hardware_interface::HardwareInfo &info) {
  node_ = std::make_shared<rclcpp::Node>("fri_hardware_interface_node");

  auto ret = configure_default(info);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SystemInterface.");
    return ret;
  }

  init_command_interfaces_();
  init_state_interfaces_();
  init_last_hw_states_();

  if (!verify_number_of_joints_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_joint_command_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_joint_state_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_sensors_()) {
    return hardware_interface::return_type::ERROR;
  }

  // read port_id and remote_host from lbr.ros2_control.xacro
  port_id_ = std::stoul(info_.hardware_parameters["port_id"]);
  info_.hardware_parameters["remote_host"] == "INADDR_ANY"
      ? remote_host_ = NULL
      : remote_host_ = info_.hardware_parameters["remote_host"].c_str();

  if (port_id_ < 30200 || port_id_ > 30209) {
    RCLCPP_ERROR(node_->get_logger(), "Expected port_id in [30200, 30209]. Found %d.", port_id_);
    return hardware_interface::return_type::ERROR;
  }

  if (!spawn_rt_layer_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!spawn_clients_()) {
    return hardware_interface::return_type::ERROR;
  }

  node_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::spin(node_);
    disconnect_();
  });

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> LBRHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const auto &lbr_fri_sensor = info_.sensors[0];
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SAMPLE_TIME, &hw_sample_time_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SESSION_STATE, &hw_session_state_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CONNECTION_QUALITY,
                                &hw_connection_quality_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SAFETY_STATE, &hw_safety_state_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_OPERATION_MODE, &hw_operation_mode_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_DRIVE_STATE, &hw_drive_state_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CLIENT_COMMAND_MODE,
                                &hw_client_command_mode_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_OVERLAY_TYPE, &hw_overlay_type_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CONTROL_MODE, &hw_control_mode_);

  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_);
  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TIME_STAMP_NANO_SEC,
                                &hw_time_stamp_nano_sec_);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                  &hw_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_JOINT_POSITION,
                                  &hw_commanded_joint_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                  &hw_effort_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_TORQUE,
                                  &hw_commanded_torque_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EXTERNAL_TORQUE,
                                  &hw_external_torque_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_IPO_JOINT_POSITION,
                                  &hw_ipo_joint_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocity_[i]);
  }

  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TRACKING_PERFORMANCE,
                                &hw_tracking_performance_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LBRHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                    &hw_position_command_[i]);

    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                    &hw_effort_command_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type LBRHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LBRHardwareInterface::start() {
  if (!connect_()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect to robot.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LBRHardwareInterface::stop() {
  if (!disconnect_()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect from robot.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LBRHardwareInterface::read() {
  auto lbr_state = *rt_lbr_state_buf_->readFromRT();

  if (!lbr_state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read lbr_state.");
    return hardware_interface::return_type::ERROR;
  }

  if (exit_commanding_active_(static_cast<KUKA::FRI::ESessionState>(hw_session_state_),
                              static_cast<KUKA::FRI::ESessionState>(lbr_state->session_state))) {
    RCLCPP_INFO(node_->get_logger(), "LBR left COMMANDING_ACTIVE. Please re-run lbr_bringup.");
    return hardware_interface::return_type::ERROR;
  }

  hw_sample_time_ = lbr_state->sample_time;
  hw_session_state_ = static_cast<double>(lbr_state->session_state);
  hw_connection_quality_ = static_cast<double>(lbr_state->connection_quality);
  hw_safety_state_ = static_cast<double>(lbr_state->safety_state);
  hw_operation_mode_ = static_cast<double>(lbr_state->operation_mode);
  hw_drive_state_ = static_cast<double>(lbr_state->drive_state);
  hw_client_command_mode_ = static_cast<double>(lbr_state->client_command_mode);
  hw_overlay_type_ = static_cast<double>(lbr_state->overlay_type);
  hw_control_mode_ = static_cast<double>(lbr_state->control_mode);

  hw_time_stamp_sec_ = static_cast<double>(lbr_state->time_stamp_sec);
  hw_time_stamp_nano_sec_ = static_cast<double>(lbr_state->time_stamp_nano_sec);

  std::copy(lbr_state->measured_joint_position.cbegin(), lbr_state->measured_joint_position.cend(),
            hw_position_.begin());
  std::copy(lbr_state->commanded_joint_position.cbegin(),
            lbr_state->commanded_joint_position.cend(), hw_commanded_joint_position_.begin());
  std::copy(lbr_state->measured_torque.cbegin(), lbr_state->measured_torque.cend(),
            hw_effort_.begin());
  std::copy(lbr_state->commanded_torque.cbegin(), lbr_state->commanded_torque.cend(),
            hw_commanded_torque_.begin());
  std::copy(lbr_state->external_torque.cbegin(), lbr_state->external_torque.cend(),
            hw_commanded_torque_.begin());
  std::copy(lbr_state->ipo_joint_position.cbegin(), lbr_state->ipo_joint_position.cend(),
            hw_ipo_joint_position_.begin());
  hw_tracking_performance_ = lbr_state->tracking_performance;
  compute_hw_velocity_();
  update_last_hw_states_();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LBRHardwareInterface::write() {
  if (rt_lbr_command_pub_->trylock()) {
    std::copy(hw_position_command_.cbegin(), hw_position_command_.cend(),
              rt_lbr_command_pub_->msg_.joint_position.begin());
    rt_lbr_command_pub_->msg_.wrench.fill(std::numeric_limits<double>::quiet_NaN());
    std::copy(hw_effort_command_.cbegin(), hw_effort_command_.cend(),
              rt_lbr_command_pub_->msg_.torque.begin());
    rt_lbr_command_pub_->unlockAndPublish();
  }
  return hardware_interface::return_type::OK;
}

template <typename ServiceT>
bool LBRHardwareInterface::wait_for_service_(
    const typename rclcpp::Client<ServiceT>::SharedPtr client, const uint8_t &attempts,
    const std::chrono::seconds &timeout) {
  bool spawned = false;
  uint8_t attempt = 0;
  while (attempt < attempts && !spawned && rclcpp::ok()) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", client->get_service_name());
    spawned = client->wait_for_service(timeout);
    ++attempt;
  }
  return spawned;
}

void LBRHardwareInterface::init_command_interfaces_() {
  hw_position_command_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_effort_command_.fill(std::numeric_limits<double>::quiet_NaN());
}

void LBRHardwareInterface::init_state_interfaces_() {
  hw_sample_time_ = std::numeric_limits<double>::quiet_NaN();
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

  hw_position_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_commanded_joint_position_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_effort_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_commanded_torque_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_external_torque_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_ipo_joint_position_.fill(std::numeric_limits<double>::quiet_NaN());
  hw_tracking_performance_ = std::numeric_limits<double>::quiet_NaN();

  hw_velocity_.fill(std::numeric_limits<double>::quiet_NaN());
}

bool LBRHardwareInterface::verify_number_of_joints_() {
  if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(node_->get_logger(), "Expected %d joints in URDF. Found %ld.",
                 KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
    return false;
  }
  return true;
}

bool LBRHardwareInterface::verify_joint_command_interfaces_() {
  // check command interfaces
  for (auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Joint %s received invalid number of command interfaces. Received %ld, expected %d.",
          joint.name.c_str(), joint.command_interfaces.size(), LBR_FRI_COMMAND_INTERFACE_SIZE);
      return false;
    }
    for (auto &ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s received invalid command interface: %s. Expected %s or %s.",
                     joint.name.c_str(), ci.name.c_str(), hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_EFFORT);
        return false;
      }
    }
  }
  return true;
}

bool LBRHardwareInterface::verify_joint_state_interfaces_() {
  // check state interfaces
  for (auto &joint : info_.joints) {
    if (joint.state_interfaces.size() != LBR_FRI_STATE_INTERFACE_SIZE) {
      RCLCPP_ERROR(
          node_->get_logger(),
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
            node_->get_logger(),
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

bool LBRHardwareInterface::verify_sensors_() {
  // check lbr specific state interfaces
  if (info_.sensors.size() > 1) {
    RCLCPP_ERROR(node_->get_logger(), "Expected 1 sensor, got %ld", info_.sensors.size());
    return false;
  }

  // check all interfaces are defined in lbr.ros2_control.xacro
  const auto &lbr_fri_sensor = info_.sensors[0];
  if (lbr_fri_sensor.state_interfaces.size() != LBR_FRI_SENSOR_SIZE) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Sensor %s received invalid state interface. Received %ld, expected %d. ",
                 lbr_fri_sensor.name.c_str(), lbr_fri_sensor.state_interfaces.size(),
                 LBR_FRI_SENSOR_SIZE);
    return false;
  }

  // check only valid interfaces are defined
  for (const auto &si : lbr_fri_sensor.state_interfaces) {
    if (si.name != HW_IF_SAMPLE_TIME && si.name != HW_IF_SESSION_STATE &&
        si.name != HW_IF_CONNECTION_QUALITY && si.name != HW_IF_SAFETY_STATE &&
        si.name != HW_IF_OPERATION_MODE && si.name != HW_IF_DRIVE_STATE &&
        si.name != HW_IF_CLIENT_COMMAND_MODE && si.name != HW_IF_OVERLAY_TYPE &&
        si.name != HW_IF_CONTROL_MODE && si.name != HW_IF_TIME_STAMP_SEC &&
        si.name != HW_IF_TIME_STAMP_NANO_SEC && si.name != HW_IF_COMMANDED_JOINT_POSITION &&
        si.name != HW_IF_COMMANDED_TORQUE && si.name != HW_IF_EXTERNAL_TORQUE &&
        si.name != HW_IF_IPO_JOINT_POSITION && si.name != HW_IF_TRACKING_PERFORMANCE) {
      RCLCPP_ERROR(node_->get_logger(), "Sensor %s received invalid state interface %s.",
                   lbr_fri_sensor.name.c_str(), si.name.c_str());

      return false;
    }
  }
  return true;
}

bool LBRHardwareInterface::spawn_rt_layer_() {
  if (!node_) {
    printf("No node provided.\n");
    return false;
  }

  try {
    rt_lbr_state_buf_ =
        std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRState::SharedPtr>>(
            nullptr);
    lbr_state_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SensorDataQoS(),
        std::bind(&LBRHardwareInterface::lbr_state_cb_, this, std::placeholders::_1));
    lbr_command_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRCommand>(
        "/lbr_command", rclcpp::SensorDataQoS());
    rt_lbr_command_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRCommand>>(
            lbr_command_pub_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn real time layer.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRHardwareInterface::spawn_clients_() {
  if (!node_) {
    printf("No node provided.\n");
    return false;
  }

  std::string app_connect_service_name = "/lbr_app/connect";
  app_connect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppConnect>(
      app_connect_service_name, rmw_qos_profile_services_default);
  RCLCPP_INFO(node_->get_logger(), "Waiting for service %s to spawn...",
              app_connect_service_name.c_str());
  if (!wait_for_service_<lbr_fri_msgs::srv::AppConnect>(app_connect_clt_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Done.");

  std::string app_disconnect_service_name = "/lbr_app/disconnect";
  app_disconnect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppDisconnect>(
      app_disconnect_service_name, rmw_qos_profile_services_default);
  RCLCPP_INFO(node_->get_logger(), "Waiting for service %s to spawn...",
              app_disconnect_service_name.c_str());
  if (!wait_for_service_<lbr_fri_msgs::srv::AppDisconnect>(app_disconnect_clt_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Done.");
  return true;
}

bool LBRHardwareInterface::exit_commanding_active_(
    const KUKA::FRI::ESessionState &previous_session_state,
    const KUKA::FRI::ESessionState &session_state) {
  if (previous_session_state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE &&
      previous_session_state != session_state) {
    return true;
  }
  return false;
}

bool LBRHardwareInterface::connect_() {
  auto connect_request = std::make_shared<lbr_fri_msgs::srv::AppConnect::Request>();
  connect_request->port_id = port_id_;
  connect_request->remote_host = remote_host_ ? remote_host_ : "";
  auto future = app_connect_clt_->async_send_request(connect_request);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request connect service %s.",
                 app_connect_clt_->get_service_name());
    return false;
  }
  auto response = future.get();
  if (!response->connected) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect.\n%s", response->message.c_str());
  }
  return response->connected;
}

bool LBRHardwareInterface::disconnect_() {
  auto disconnect_request = std::make_shared<lbr_fri_msgs::srv::AppDisconnect::Request>();
  auto future = app_disconnect_clt_->async_send_request(disconnect_request);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request disconnect service %s.",
                 app_disconnect_clt_->get_service_name());
    return false;
  }
  auto response = future.get();
  if (!response->disconnected) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect.\n%s", response->message.c_str());
  }
  return response->disconnected;
}

void LBRHardwareInterface::lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
  rt_lbr_state_buf_->writeFromNonRT(lbr_state);
}

double LBRHardwareInterface::time_stamps_to_sec_(const double &sec, const double &nano_sec) const {
  return sec + nano_sec / 1.e9;
}

void LBRHardwareInterface::init_last_hw_states_() {
  last_hw_position_.fill(std::numeric_limits<double>::quiet_NaN());
  last_hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  last_hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();
}

void LBRHardwareInterface::update_last_hw_states_() {
  last_hw_position_ = hw_position_;
  last_hw_time_stamp_sec_ = hw_time_stamp_sec_;
  last_hw_time_stamp_nano_sec_ = hw_time_stamp_nano_sec_;
}

void LBRHardwareInterface::compute_hw_velocity_() {
  // state uninitialized
  if (std::isnan(last_hw_time_stamp_nano_sec_) || std::isnan(last_hw_position_[0])) {
    return;
  }

  // state wasn't updated
  if (last_hw_time_stamp_sec_ == hw_time_stamp_sec_ &&
      last_hw_time_stamp_nano_sec_ == hw_time_stamp_nano_sec_) {
    return;
  }

  for (uint8_t i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
    hw_velocity_[i] = (hw_position_[i] - last_hw_position_[i]) /
                      (time_stamps_to_sec_(hw_time_stamp_sec_, hw_time_stamp_nano_sec_) -
                       time_stamps_to_sec_(last_hw_time_stamp_sec_, last_hw_time_stamp_nano_sec_));
  }
}

} // end of namespace lbr_hardware_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lbr_hardware_interface::LBRHardwareInterface,
                       hardware_interface::SystemInterface)
