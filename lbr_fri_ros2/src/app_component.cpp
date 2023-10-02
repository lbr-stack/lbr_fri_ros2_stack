#include "app_component.hpp"

namespace lbr_fri_ros2 {
AppComponent::AppComponent(const rclcpp::NodeOptions &options) {
  app_node_ = std::make_shared<rclcpp::Node>("app", options);

  app_node_->declare_parameter("rt_prio", 80);
  app_node_->declare_parameter("port_id", 30200);
  app_node_->declare_parameter("remote_host", std::string(""));

  client_ptr_ = std::make_shared<Client>(app_node_);
  app_ptr_ = std::make_unique<App>(app_node_, client_ptr_);

  // default connect
  connect_(app_node_->get_parameter("port_id").as_int(),
           app_node_->get_parameter("remote_host").as_string().empty()
               ? NULL
               : app_node_->get_parameter("remote_host").as_string().c_str(),
           app_node_->get_parameter("rt_prio").as_int());

  // services
  app_connect_srv_ = app_node_->create_service<lbr_fri_msgs::srv::AppConnect>(
      "app/connect", std::bind(&AppComponent::on_app_connect_, this, std::placeholders::_1,
                               std::placeholders::_2));
  app_disconnect_srv_ = app_node_->create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "app/disconnect", std::bind(&AppComponent::on_app_disconnect_, this, std::placeholders::_1,
                                  std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
AppComponent::get_node_base_interface() const {
  return app_node_->get_node_base_interface();
}

void AppComponent::connect_(const int &port_id, const char *const remote_host,
                            const uint8_t &rt_prio, const uint8_t &max_attempts) {
  if (!app_ptr_->open_udp_socket(port_id, remote_host)) {
    return;
  };
  app_ptr_->run(rt_prio);
  uint8_t attempt = 0;
  while (!client_ptr_->get_state_interface().is_initialized() && rclcpp::ok()) {
    RCLCPP_INFO(app_node_->get_logger(), "Waiting for robot heartbeat %d/%d.", attempt + 1,
                max_attempts);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (++attempt >= max_attempts) {
      app_ptr_->close_udp_socket();
      RCLCPP_ERROR(app_node_->get_logger(), "Failed to connect to robot on max attempts.");
      return;
    }
  }
  RCLCPP_INFO(app_node_->get_logger(), "Robot connected.");

  RCLCPP_INFO(
      app_node_->get_logger(), "Control mode: %s.",
      EnumMaps::control_mode_map(client_ptr_->get_state_interface().get_state().control_mode)
          .c_str());
  RCLCPP_INFO(app_node_->get_logger(), "Sample time: %f s.",
              client_ptr_->get_state_interface().get_state().sample_time);

  // publisher
  state_pub_ = app_node_->create_publisher<lbr_fri_msgs::msg::LBRState>("state", 1);
  state_pub_timer_ = app_node_->create_wall_timer(
      std::chrono::milliseconds(
          static_cast<int64_t>(client_ptr_->get_state_interface().get_state().sample_time * 1.e3)),
      std::bind(&AppComponent::on_state_pub_timer_, this));

  // await commanding active thread
  std::thread await_commanding_active_thread([this]() {
    while (client_ptr_->get_state_interface().get_state().session_state !=
               KUKA::FRI::ESessionState::COMMANDING_ACTIVE &&
           rclcpp::ok()) {
      RCLCPP_INFO(app_node_->get_logger(), "Waiting for robot to enter %s state.",
                  EnumMaps::session_state_map(KUKA::FRI::ESessionState::COMMANDING_ACTIVE).c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    RCLCPP_INFO(app_node_->get_logger(), "Client command mode: %s.",
                EnumMaps::client_command_mode_map(
                    client_ptr_->get_state_interface().get_state().client_command_mode)
                    .c_str());

    // subscriptions
    switch (client_ptr_->get_state_interface().get_state().client_command_mode) {
    case KUKA::FRI::EClientCommandMode::POSITION:
      position_command_sub_ = app_node_->create_subscription<lbr_fri_msgs::msg::LBRPositionCommand>(
          "command/position", 1,
          std::bind(&AppComponent::on_position_command_, this, std::placeholders::_1));
      break;
    case KUKA::FRI::EClientCommandMode::TORQUE:
      torque_command_sub_ = app_node_->create_subscription<lbr_fri_msgs::msg::LBRTorqueCommand>(
          "command/torque", 1,
          std::bind(&AppComponent::on_torque_command_, this, std::placeholders::_1));
      break;
    case KUKA::FRI::EClientCommandMode::WRENCH:
      wrench_command_sub_ = app_node_->create_subscription<lbr_fri_msgs::msg::LBRWrenchCommand>(
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

  if (client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_position_command->joint_position;
    client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not commanding active, reset
  lbr_command_.joint_position =
      client_ptr_->get_state_interface().get_state().measured_joint_position;
}

void AppComponent::on_torque_command_(
    const lbr_fri_msgs::msg::LBRTorqueCommand::SharedPtr lbr_torque_command) {
  if (!on_command_checks_(KUKA::FRI::EClientCommandMode::TORQUE)) {
    return;
  }

  if (client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_torque_command->joint_position;
    lbr_command_.torque = lbr_torque_command->torque;
    client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not active, reset
  lbr_command_.joint_position =
      client_ptr_->get_state_interface().get_state().measured_joint_position;
  std::fill(lbr_command_.torque.begin(), lbr_command_.torque.end(), 0.0);
}

void AppComponent::on_wrench_command_(
    const lbr_fri_msgs::msg::LBRWrenchCommand::SharedPtr lbr_wrench_command) {
  if (!on_command_checks_(KUKA::FRI::EClientCommandMode::WRENCH)) {
    return;
  }

  if (client_ptr_->get_state_interface().get_state().session_state ==
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    lbr_command_.joint_position = lbr_wrench_command->joint_position;
    lbr_command_.wrench = lbr_wrench_command->wrench;
    client_ptr_->get_command_interface().set_command_target(lbr_command_);
    return;
  }

  // if not active, reset
  lbr_command_.joint_position =
      client_ptr_->get_state_interface().get_state().measured_joint_position;
  std::fill(lbr_command_.wrench.begin(), lbr_command_.wrench.end(), 0.0);
}

bool AppComponent::on_command_checks_(const int &expected_command_mode) {
  if (!client_ptr_) {
    RCLCPP_ERROR(app_node_->get_logger(), "Client not configured.");
    return false;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode ==
      KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE) {
    return false;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode != expected_command_mode) {
    RCLCPP_ERROR(app_node_->get_logger(), "Wrench command only allowed in wrench command mode.");
    return false;
  }
  return true;
}

void AppComponent::on_state_pub_timer_() {
  state_pub_->publish(client_ptr_->get_state_interface().get_state());
}

void AppComponent::on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                                   lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_->get_logger(),
              "Connecting to robot via service. Port ID: %d, remote host: '%s'.", request->port_id,
              request->remote_host.c_str());
  connect_(request->port_id, request->remote_host.empty() ? NULL : request->remote_host.c_str(),
           request->rt_prio, request->max_attempts);
  response->connected = client_ptr_->get_state_interface().is_initialized();
  response->message = response->connected ? "Robot connected." : "Failed.";
}

void AppComponent::on_app_disconnect_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_->get_logger(), "Disconnecting from robot via service.");
  app_ptr_->stop_run();
  response->disconnected = app_ptr_->close_udp_socket();
  state_pub_timer_.reset(nullptr);
  state_pub_.reset(nullptr);
  position_command_sub_.reset(nullptr);
  torque_command_sub_.reset(nullptr);
  wrench_command_sub_.reset(nullptr);
  response->message = response->disconnected ? "Robot disconnected." : "Failed.";
  RCLCPP_INFO(app_node_->get_logger(), response->message.c_str());
}
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::AppComponent)
