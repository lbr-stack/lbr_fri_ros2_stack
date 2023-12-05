#include "lbr_ros2_control/lbr_estimated_ft_broadcaster.hpp"

namespace lbr_ros2_control {
LBREstimatedFTBroadcaster::LBREstimatedFTBroadcaster()
    : jacobian_(6, KUKA::FRI::LBRState::NUMBER_OF_JOINTS),
      jacobian_pinv_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 6) {}

controller_interface::InterfaceConfiguration
LBREstimatedFTBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LBREstimatedFTBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + HW_IF_EXTERNAL_TORQUE);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn LBREstimatedFTBroadcaster::on_init() {
  try {
    wrench_stamped_publisher_ptr_ =
        this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "estimated_force_torque", rclcpp::SensorDataQoS());
    rt_wrench_stamped_publisher_ptr_ =
        std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
            wrench_stamped_publisher_ptr_);
    kinematics_interface_kdl_.initialize(this->get_node()->get_node_parameters_interface(),
                                         end_effector_link_);
    if (!this->get_node()->get_parameter_or<double>("damping", damping_, 2e-1)) {
      RCLCPP_WARN(this->get_node()->get_logger(),
                  "Failed to get damping parameter, using default value: %f.", damping_);
    }
    if (joint_names_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
      RCLCPP_ERROR(
          this->get_node()->get_logger(),
          "Number of joint names (%ld) does not match the number of joints in the robot (%d).",
          joint_names_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR virtual FT broadcaster with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBREstimatedFTBroadcaster::update(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
  // check any for nan
  if (std::isnan(joint_position_state_interfaces_[0].get().get_value())) {
    return controller_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    joint_positions_(i) = joint_position_state_interfaces_[i].get().get_value();
    external_joint_torques_(i) = external_joint_torque_state_interfaces_[i].get().get_value();
  }
  // compute virtual FT given Jacobian and external joint torques
  kinematics_interface_kdl_.calculate_jacobian(joint_positions_, end_effector_link_, jacobian_);
  jacobian_pinv_ = damped_least_squares_(jacobian_, damping_);
  virtual_ft_ = jacobian_pinv_.transpose() * external_joint_torques_;
  // publish
  if (rt_wrench_stamped_publisher_ptr_->trylock()) {
    rt_wrench_stamped_publisher_ptr_->msg_.header.stamp = this->get_node()->now();
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.x = virtual_ft_(0);
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.y = virtual_ft_(1);
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.z = virtual_ft_(2);
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.x = virtual_ft_(3);
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.y = virtual_ft_(4);
    rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.z = virtual_ft_(5);
    rt_wrench_stamped_publisher_ptr_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
LBREstimatedFTBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBREstimatedFTBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  init_states_();
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBREstimatedFTBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_state_interfaces_();
  return controller_interface::CallbackReturn::SUCCESS;
}

void LBREstimatedFTBroadcaster::init_states_() {
  jacobian_.setConstant(std::numeric_limits<double>::quiet_NaN());
  jacobian_pinv_.setConstant(std::numeric_limits<double>::quiet_NaN());
  joint_positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_joint_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  virtual_ft_.setConstant(std::numeric_limits<double>::quiet_NaN());
  rt_wrench_stamped_publisher_ptr_->msg_.header.frame_id = end_effector_link_;
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.x = virtual_ft_(0);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.y = virtual_ft_(1);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.z = virtual_ft_(2);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.x = virtual_ft_(3);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.y = virtual_ft_(4);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.z = virtual_ft_(5);
}

bool LBREstimatedFTBroadcaster::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.emplace_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_EXTERNAL_TORQUE) {
      external_joint_torque_state_interfaces_.emplace_back(std::ref(state_interface));
    }
  }
  if (joint_position_state_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces (%ld) does not match the number of joints "
        "in the robot (%d).",
        joint_position_state_interfaces_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  if (external_joint_torque_state_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of external joint torque state interfaces (%ld) does not match the number "
                 "of joints "
                 "in the robot (%d).",
                 external_joint_torque_state_interfaces_.size(),
                 KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return false;
  }
  return true;
}

void LBREstimatedFTBroadcaster::clear_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  external_joint_torque_state_interfaces_.clear();
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
LBREstimatedFTBroadcaster::damped_least_squares_(
    const MatT &mat,
    typename MatT::Scalar lambda) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    dampedSingularValuesInv(i, i) =
        singularValues(i) / (singularValues(i) * singularValues(i) + lambda * lambda);
  }
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
}
} // end of namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBREstimatedFTBroadcaster,
                       controller_interface::ControllerInterface)
