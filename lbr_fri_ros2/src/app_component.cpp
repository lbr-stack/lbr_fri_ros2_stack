#include "app_component.hpp"

namespace lbr_fri_ros2 {
AppComponent::AppComponent(const rclcpp::NodeOptions &options) {
  app_node_ = std::make_shared<rclcpp::Node>("app", options);

  app_node_->declare_parameter("rt_prio", 80);
  app_node_->declare_parameter("port_id", 30200);
  app_node_->declare_parameter("remote_host", std::string(""));

  client_ptr_ = std::make_shared<Client>(app_node_);
  app_ptr_ = std::make_unique<App>(app_node_, client_ptr_);
  app_ptr_->open_udp_socket(app_node_->get_parameter("port_id").as_int(),
                            app_node_->get_parameter("remote_host").as_string().empty()
                                ? NULL
                                : app_node_->get_parameter("remote_host").as_string().c_str());
  app_ptr_->run(app_node_->get_parameter("rt_prio").as_int());
  while (!client_ptr_->get_state_interface().is_initialized()) {
    RCLCPP_INFO(app_node_->get_logger(), "Waiting for robot heartbeat.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO(app_node_->get_logger(), "Robot connected.");

  // publisher
  state_pub_ = app_node_->create_publisher<lbr_fri_msgs::msg::LBRState>("/lbr/state", 1);
  state_pub_timer_ = app_node_->create_wall_timer(
      std::chrono::milliseconds(
          static_cast<int64_t>(client_ptr_->get_state_interface().get_state().sample_time * 1.e3)),
      std::bind(&AppComponent::on_state_pub_timer_, this));

  // subscriptions
  position_command_sub_ =
      app_node_->create_subscription<lbr_fri_msgs::msg::LBRPositionCommand>( // TODO: fix namespaces
          "/lbr/command/position", 1,
          std::bind(&AppComponent::on_position_command_, this, std::placeholders::_1));
  torque_command_sub_ = app_node_->create_subscription<lbr_fri_msgs::msg::LBRTorqueCommand>(
      "/lbr/command/torque", 1,
      std::bind(&AppComponent::on_torque_command_, this, std::placeholders::_1));
  wrench_command_sub_ = app_node_->create_subscription<lbr_fri_msgs::msg::LBRWrenchCommand>(
      "/lbr/command/wrench", 1,
      std::bind(&AppComponent::on_wrench_command_, this, std::placeholders::_1));

  // services
  app_connect_srv_ = app_node_->create_service<lbr_fri_msgs::srv::AppConnect>(
      "/lbr/app/connect", std::bind(&AppComponent::on_app_connect_, this, std::placeholders::_1,
                                    std::placeholders::_2));
  app_disconnect_srv_ = app_node_->create_service<lbr_fri_msgs::srv::AppDisconnect>(
      "/lbr/app/disconnect", std::bind(&AppComponent::on_app_disconnect_, this,
                                       std::placeholders::_1, std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
AppComponent::get_node_base_interface() const {
  return app_node_->get_node_base_interface();
}

void AppComponent::on_position_command_(
    const lbr_fri_msgs::msg::LBRPositionCommand::SharedPtr lbr_position_command) {
  if (!client_ptr_) {
    RCLCPP_ERROR(app_node_->get_logger(), "Client not configured.");
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode ==
      KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE) {
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode !=
      KUKA::FRI::EClientCommandMode::POSITION) {
    RCLCPP_ERROR(app_node_->get_logger(),
                 "Joint position command only allowed in position command mode.");
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
  if (!client_ptr_) {
    RCLCPP_ERROR(app_node_->get_logger(), "Client not configured.");
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode ==
      KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE) {
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode !=
      KUKA::FRI::EClientCommandMode::TORQUE) {
    RCLCPP_ERROR(app_node_->get_logger(), "Torque command only allowed in torque command mode.");
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
  if (!client_ptr_) {
    RCLCPP_ERROR(app_node_->get_logger(), "Client not configured.");
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode ==
      KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE) {
    return;
  }
  if (client_ptr_->get_state_interface().get_state().client_command_mode !=
      KUKA::FRI::EClientCommandMode::WRENCH) {
    RCLCPP_ERROR(app_node_->get_logger(), "Wrench command only allowed in wrench command mode.");
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

void AppComponent::on_state_pub_timer_() {
  state_pub_->publish(client_ptr_->get_state_interface().get_state());
}

void AppComponent::on_app_connect_(const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
                                   lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_->get_logger(),
              "Connecting to robot via service. Port ID: %d, remote host: '%s'.", request->port_id,
              request->remote_host.c_str());
  response->connected = app_ptr_->open_udp_socket(
      request->port_id, request->remote_host.empty() ? NULL : request->remote_host.c_str());
  if (response->connected) {
    app_ptr_->run(app_node_->get_parameter("rt_prio").as_int());
  }
  response->message = response->connected ? "Robot connected." : "Failed.";
  RCLCPP_INFO(app_node_->get_logger(), response->message.c_str());
}

void AppComponent::on_app_disconnect_(
    const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
    lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response) {
  RCLCPP_INFO(app_node_->get_logger(), "Disconnecting from robot via service.");
  app_ptr_->stop_run();
  response->disconnected = app_ptr_->close_udp_socket();
  response->message = response->disconnected ? "Robot disconnected." : "Failed.";
  RCLCPP_INFO(app_node_->get_logger(), response->message.c_str());
}
} // end of namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::AppComponent)
