#include "app_component.hpp"

namespace lbr_fri_ros2 {
AppComponent::AppComponent(const rclcpp::NodeOptions &options) {
  app_node_ptr_ = std::make_shared<rclcpp::Node>("app", options);

  app_node_ptr_->declare_parameter("port_id", 30200);
  app_node_ptr_->declare_parameter("remote_host", std::string(""));
  app_node_ptr_->declare_parameter("rt_prio", 80);
  app_node_ptr_->declare_parameter("robot_description", std::string(""));
  app_node_ptr_->declare_parameter("pid.p", 1.0);
  app_node_ptr_->declare_parameter("pid.i", 0.0);
  app_node_ptr_->declare_parameter("pid.d", 0.0);
  app_node_ptr_->declare_parameter("pid.i_max", 0.0);
  app_node_ptr_->declare_parameter("pid.i_min", 0.0);
  app_node_ptr_->declare_parameter("pid.antiwindup", false);
  app_node_ptr_->declare_parameter("command_guard_variant", std::string("safe_stop"));
  app_node_ptr_->declare_parameter("external_torque_cutoff_frequency", 10.);
  app_node_ptr_->declare_parameter("measured_torque_cutoff_frequency", 10.);
  app_node_ptr_->declare_parameter("open_loop", true);

  // prepare parameters
  PIDParameters pid_parameters;
  pid_parameters.p = app_node_ptr_->get_parameter("pid.p").as_double();
  pid_parameters.i = app_node_ptr_->get_parameter("pid.i").as_double();
  pid_parameters.d = app_node_ptr_->get_parameter("pid.d").as_double();
  pid_parameters.i_max = app_node_ptr_->get_parameter("pid.i_max").as_double();
  pid_parameters.i_min = app_node_ptr_->get_parameter("pid.i_min").as_double();
  pid_parameters.antiwindup = app_node_ptr_->get_parameter("pid.antiwindup").as_bool();
  CommandGuardParameters command_guard_parameters;
  std::string command_guard_variant =
      app_node_ptr_->get_parameter("command_guard_variant").as_string();
  StateInterfaceParameters state_interface_parameters;
  state_interface_parameters.external_torque_cutoff_frequency =
      app_node_ptr_->get_parameter("external_torque_cutoff_frequency").as_double();
  state_interface_parameters.measured_torque_cutoff_frequency =
      app_node_ptr_->get_parameter("measured_torque_cutoff_frequency").as_double();
  bool open_loop = app_node_ptr_->get_parameter("open_loop").as_bool();

  // load robot description and parse limits
  auto robot_description_param = app_node_ptr_->get_parameter("robot_description");
  urdf::Model model;
  if (!model.initString(robot_description_param.as_string())) {
    std::string err = "Failed to intialize urdf model from '" + robot_description_param.get_name() +
                      "' parameter.";
    RCLCPP_ERROR(app_node_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }

  std::size_t jnt_cnt = 0;
  for (const auto &name_joint_pair : model.joints_) {
    const auto joint = name_joint_pair.second;
    if (joint->type == urdf::Joint::REVOLUTE) {
      if (jnt_cnt >= command_guard_parameters.joint_names.size()) {
        std::string errs =
            "Found too many joints in '" + robot_description_param.get_name() + "' parameter.";
        RCLCPP_ERROR(app_node_ptr_->get_logger(), errs.c_str());
        throw std::runtime_error(errs);
      }
      command_guard_parameters.joint_names[jnt_cnt] = name_joint_pair.first;
      command_guard_parameters.min_position[jnt_cnt] = joint->limits->lower;
      command_guard_parameters.max_position[jnt_cnt] = joint->limits->upper;
      command_guard_parameters.max_velocity[jnt_cnt] = joint->limits->velocity;
      command_guard_parameters.max_torque[jnt_cnt] = joint->limits->effort;
      ++jnt_cnt;
    }
  }

  // configure client
  async_client_ptr_ =
      std::make_shared<AsyncClient>(pid_parameters, command_guard_parameters, command_guard_variant,
                                    state_interface_parameters, open_loop);
  app_ptr_ = std::make_unique<App>(async_client_ptr_);

  // default connect
  connect_(app_node_ptr_->get_parameter("port_id").as_int(),
           app_node_ptr_->get_parameter("remote_host").as_string().empty()
               ? NULL
               : app_node_ptr_->get_parameter("remote_host").as_string().c_str(),
           app_node_ptr_->get_parameter("rt_prio").as_int());

  // services
  app_connect_srv_ = app_node_ptr_->create_service<lbr_fri_msgs::srv::AppConnect>(
      "app/connect", std::bind(&AppComponent::on_app_connect_, this, std::placeholders::_1,
                               std::placeholders::_2));
  app_disconnect_srv_ = app_node_ptr_->create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "app/disconnect", std::bind(&AppComponent::on_app_disconnect_, this, std::placeholders::_1,
                                  std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
AppComponent::get_node_base_interface() const {
  return app_node_ptr_->get_node_base_interface();
}

void AppComponent::connect_(const int &port_id, const char *const remote_host,
                            const uint8_t &rt_prio, const uint8_t &max_attempts) {
  if (!app_ptr_->open_udp_socket(port_id, remote_host)) {
    return;
  };
  app_ptr_->run(rt_prio);
  uint8_t attempt = 0;
  while (!async_client_ptr_->get_state_interface().is_initialized() && rclcpp::ok()) {
    RCLCPP_INFO(app_node_ptr_->get_logger(), "Waiting for robot heartbeat [%d/%d]. Port ID: %d.",
                attempt + 1, max_attempts, port_id);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (++attempt >= max_attempts) {
      app_ptr_->close_udp_socket();
      RCLCPP_ERROR(app_node_ptr_->get_logger(), "Failed to connect to robot on max attempts.");
      return;
    }
  }
  RCLCPP_INFO(app_node_ptr_->get_logger(), "Robot connected.");

  RCLCPP_INFO(
      app_node_ptr_->get_logger(), "Control mode: '%s'.",
      EnumMaps::control_mode_map(async_client_ptr_->get_state_interface().get_state().control_mode)
          .c_str());
  RCLCPP_INFO(app_node_ptr_->get_logger(), "Sample time: %.3f s / %.1f Hz.",
              async_client_ptr_->get_state_interface().get_state().sample_time,
              1. / async_client_ptr_->get_state_interface().get_state().sample_time);

  // publisher
  state_pub_ = app_node_ptr_->create_publisher<lbr_fri_msgs::msg::LBRState>("state", 1);
  state_pub_timer_ = app_node_ptr_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(
          async_client_ptr_->get_state_interface().get_state().sample_time * 1.e3)),
      std::bind(&AppComponent::on_state_pub_timer_, this));

  // await commanding active thread
  std::thread await_commanding_active_thread([this]() {
    while (async_client_ptr_->get_state_interface().get_state().session_state !=
               KUKA::FRI::ESessionState::COMMANDING_ACTIVE &&
           rclcpp::ok()) {
      RCLCPP_INFO(app_node_ptr_->get_logger(), "Waiting for robot to enter '%s' state.",
                  EnumMaps::session_state_map(KUKA::FRI::ESessionState::COMMANDING_ACTIVE).c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    RCLCPP_INFO(app_node_ptr_->get_logger(), "AsyncClient command mode: '%s'.",
                EnumMaps::client_command_mode_map(
                    async_client_ptr_->get_state_interface().get_state().client_command_mode)
                    .c_str());

    // subscriptions
    switch (async_client_ptr_->get_state_interface().get_state().client_command_mode) {
    case KUKA::FRI::EClientCommandMode::POSITION:
      position_command_sub_ =
          app_node_ptr_->create_subscription<lbr_fri_msgs::msg::LBRPositionCommand>(
              "command/joint_position", 1,
              std::bind(&AppComponent::on_position_command_, this, std::placeholders::_1));
      break;
    case KUKA::FRI::EClientCommandMode::TORQUE:
      torque_command_sub_ = app_node_ptr_->create_subscription<lbr_fri_msgs::msg::LBRTorqueCommand>(
          "command/torque", 1,
          std::bind(&AppComponent::on_torque_command_, this, std::placeholders::_1));
      break;
    case KUKA::FRI::EClientCommandMode::WRENCH:
      wrench_command_sub_ = app_node_ptr_->create_subscription<lbr_fri_msgs::msg::LBRWrenchCommand>(
          "command/wrench", 1,
          std::bind(&AppComponent::on_wrench_command_, this, std::placeholders::_1));
      break;
    default:
      break;
    }
  });
  await_commanding_active_thread.detach();
}

void AppComponent::on_position_command_(
    const lbr_fri_msgs::msg::LBRPositionCommand::SharedPtr lbr_position_command) {
  if (!on_command_checks_(KUKA::FRI::EClientCommandMode::POSITION)) {
    return;
  }

  if (async_client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_position_command->joint_position;
    async_client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not commanding active, reset
  lbr_command_.joint_position =
      async_client_ptr_->get_state_interface().get_state().measured_joint_position;
}

void AppComponent::on_torque_command_(
    const lbr_fri_msgs::msg::LBRTorqueCommand::SharedPtr lbr_torque_command) {
  if (!on_command_checks_(KUKA::FRI::EClientCommandMode::TORQUE)) {
    return;
  }

  if (async_client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_torque_command->joint_position;
    lbr_command_.torque = lbr_torque_command->torque;
    async_client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not active, reset
  lbr_command_.joint_position =
      async_client_ptr_->get_state_interface().get_state().measured_joint_position;
  std::fill(lbr_command_.torque.begin(), lbr_command_.torque.end(), 0.0);
}

void AppComponent::on_wrench_command_(
    const lbr_fri_msgs::msg::LBRWrenchCommand::SharedPtr lbr_wrench_command) {
  if (!on_command_checks_(KUKA::FRI::EClientCommandMode::WRENCH)) {
    return;
  }

  if (async_client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_wrench_command->joint_position;
    lbr_command_.wrench = lbr_wrench_command->wrench;
    async_client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not active, reset
  lbr_command_.joint_position =
      async_client_ptr_->get_state_interface().get_state().measured_joint_position;
  std::fill(lbr_command_.wrench.begin(), lbr_command_.wrench.end(), 0.0);
}

bool AppComponent::on_command_checks_(const int &expected_command_mode) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR(app_node_ptr_->get_logger(), "AsyncClient not configured.");
    return false;
  }
  if (async_client_ptr_->get_state_interface().get_state().client_command_mode ==
      KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE) {
    return false;
  }
  if (async_client_ptr_->get_state_interface().get_state().client_command_mode !=
      expected_command_mode) {
    RCLCPP_ERROR(app_node_ptr_->get_logger(),
                 "Wrench command only allowed in wrench command mode.");
    return false;
  }
  return true;
}

void AppComponent::on_state_pub_timer_() {
  state_pub_->publish(async_client_ptr_->get_state_interface().get_state());
}

void AppComponent::on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                                   lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_ptr_->get_logger(),
              "Connecting to robot via service. Port ID: %d, remote host: '%s'.", request->port_id,
              request->remote_host.c_str());
  connect_(request->port_id, request->remote_host.empty() ? NULL : request->remote_host.c_str(),
           request->rt_prio, request->max_attempts);
  response->connected = async_client_ptr_->get_state_interface().is_initialized();
  response->message = response->connected ? "Robot connected." : "Failed.";
}

void AppComponent::on_app_disconnect_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_ptr_->get_logger(), "Disconnecting from robot via service.");
  app_ptr_->stop_run();
  response->disconnected = app_ptr_->close_udp_socket();
  state_pub_timer_.reset();
  state_pub_.reset();
  position_command_sub_.reset();
  torque_command_sub_.reset();
  wrench_command_sub_.reset();
  response->message = response->disconnected ? "Robot disconnected." : "Failed.";
  RCLCPP_INFO(app_node_ptr_->get_logger(), response->message.c_str());
}
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::AppComponent)
