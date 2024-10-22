#include "lbr_ros2_control/controllers/twist_controller.hpp"

namespace lbr_ros2_control {
TwistImpl::TwistImpl(const std::string &robot_description, const TwistParameters &parameters)
    : parameters_(parameters) {
  kinematics_ptr_ = std::make_unique<lbr_fri_ros2::Kinematics>(
      robot_description, parameters_.chain_root, parameters_.chain_tip);
}

void TwistImpl::compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target,
                        lbr_fri_ros2::const_jnt_array_t_ref q, lbr_fri_ros2::jnt_array_t_ref dq) {
  // twist to Eigen
  twist_target_[0] = twist_target->linear.x;
  twist_target_[1] = twist_target->linear.y;
  twist_target_[2] = twist_target->linear.z;
  twist_target_[3] = twist_target->angular.x;
  twist_target_[4] = twist_target->angular.y;
  twist_target_[5] = twist_target->angular.z;

  // clip velocity
  twist_target_.head(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_linear_velocity, parameters_.max_linear_velocity);
  });
  twist_target_.tail(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_angular_velocity, parameters_.max_angular_velocity);
  });

  // compute jacobian
  auto jacobian = kinematics_ptr_->compute_jacobian(q);
  jacobian_inv_ = lbr_fri_ros2::pinv(jacobian.data, parameters_.damping);

  // compute target joint veloctiy and map it to dq
  Eigen::Map<Eigen::Matrix<double, lbr_fri_ros2::N_JNTS, 1>>(dq.data()) =
      jacobian_inv_ * twist_target_;
}

TwistController::TwistController() : rt_twist_ptr_(nullptr), twist_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
TwistController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
TwistController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SAMPLE_TIME);
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SESSION_STATE);
  return interface_configuration;
}

controller_interface::CallbackReturn TwistController::on_init() {
  try {
    twist_subscription_ptr_ = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "command/twist", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          rt_twist_ptr_.writeFromNonRT(msg);
          updates_since_last_command_ = 0;
        });
    this->get_node()->declare_parameter("robot_name", "lbr");
    this->get_node()->declare_parameter("chain_root", "lbr_link_0");
    this->get_node()->declare_parameter("chain_tip", "lbr_link_ee");
    this->get_node()->declare_parameter("damping", 0.2);
    this->get_node()->declare_parameter("max_linear_velocity", 0.1);
    this->get_node()->declare_parameter("max_angular_velocity", 0.1);
    configure_joint_names_();
    configure_twist_impl_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to initialize twist controller with: %s.",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TwistController::update(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration &period) {
  auto twist_command = rt_twist_ptr_.readFromRT();
  if (!twist_command || !(*twist_command)) {
    return controller_interface::return_type::OK;
  }
  if (!twist_impl_ptr_) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Twist controller not initialized.");
    return controller_interface::return_type::ERROR;
  }
  if (static_cast<int>(session_state_interface_->get().get_value()) !=
      KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    return controller_interface::return_type::OK;
  }
  if (updates_since_last_command_ >
      static_cast<int>(max_time_without_command_ / period.seconds())) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "No twist command received within time %f. Stopping the controller.",
                 max_time_without_command_);
    return controller_interface::return_type::ERROR;
  }

  // pass joint positions to q_
  std::for_each(q_.begin(), q_.end(), [&, i = 0](double &q_i) mutable {
    q_i = this->state_interfaces_[i].get_value();
    ++i;
  });

  // compute the joint velocity from the twist command target
  twist_impl_ptr_->compute(*twist_command, q_, dq_);

  // pass joint positions to hardware
  std::for_each(q_.begin(), q_.end(), [&, i = 0](const double &q_i) mutable {
    this->command_interfaces_[i].set_value(
        q_i + dq_[i] * sample_time_state_interface_->get().get_value());
    ++i;
  });

  ++updates_since_last_command_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
TwistController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  reset_command_buffer_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_state_interfaces_();
  reset_command_buffer_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool TwistController::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.emplace_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SAMPLE_TIME) {
      sample_time_state_interface_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SESSION_STATE) {
      session_state_interface_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  return true;
}

void TwistController::clear_state_interfaces_() { joint_position_state_interfaces_.clear(); }

void TwistController::reset_command_buffer_() {
  rt_twist_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>>(nullptr);
};

void TwistController::configure_joint_names_() {
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

void TwistController::configure_twist_impl_() {
  twist_impl_ptr_ = std::make_unique<TwistImpl>(
      this->get_robot_description(),
      TwistParameters{this->get_node()->get_parameter("chain_root").as_string(),
                      this->get_node()->get_parameter("chain_tip").as_string(),
                      this->get_node()->get_parameter("damping").as_double(),
                      this->get_node()->get_parameter("max_linear_velocity").as_double(),
                      this->get_node()->get_parameter("max_angular_velocity").as_double()});
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::TwistController, controller_interface::ControllerInterface)
