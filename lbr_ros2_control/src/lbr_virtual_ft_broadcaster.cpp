#include "lbr_ros2_control/lbr_virtual_ft_broadcaster.hpp"

namespace lbr_ros2_control {
LBRVirtualFTBroadcaster::LBRVirtualFTBroadcaster()
    : jacobian_(6, KUKA::FRI::LBRState::NUMBER_OF_JOINTS),
      jacobian_pinv_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 6) {}

controller_interface::InterfaceConfiguration
LBRVirtualFTBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LBRVirtualFTBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    interface_configuration.names.push_back(joint_name + "/" + HW_IF_EXTERNAL_TORQUE);
  }
  return interface_configuration;
}

controller_interface::CallbackReturn LBRVirtualFTBroadcaster::on_init() {
  try {
    wrench_stamped_publisher_ptr_ =
        this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "wrench", rclcpp::SensorDataQoS());
    rt_wrench_stamped_publisher_ptr_ =
        std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
            wrench_stamped_publisher_ptr_);
    kinematics_interface_kdl_.initialize(this->get_node()->get_node_parameters_interface(),
                                         end_effector_link_);
    if (!this->get_node()->get_parameter_or<double>("damping", damping_, 2e-1)) {
      RCLCPP_WARN(this->get_node()->get_logger(),
                  "Failed to get damping parameter, using default value: %f.", damping_);
    }
    for (auto &state_interface : state_interfaces_) {
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        joint_position_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_interface_name() == HW_IF_EXTERNAL_TORQUE) {
        external_joint_torque_interfaces_.emplace_back(std::ref(state_interface));
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR virtual FT broadcaster with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRVirtualFTBroadcaster::update(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    joint_positions_(i) = joint_position_interfaces_[i].get().get_value();
    external_joint_torques_(i) = external_joint_torque_interfaces_[i].get().get_value();
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
LBRVirtualFTBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualFTBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  init_state_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualFTBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void LBRVirtualFTBroadcaster::init_state_() {
  jacobian_.setConstant(std::numeric_limits<double>::quiet_NaN());
  jacobian_pinv_.setConstant(std::numeric_limits<double>::quiet_NaN());
  joint_positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_joint_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  virtual_ft_.setConstant(std::numeric_limits<double>::quiet_NaN());
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.x = virtual_ft_(0);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.y = virtual_ft_(1);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.force.z = virtual_ft_(2);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.x = virtual_ft_(3);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.y = virtual_ft_(4);
  rt_wrench_stamped_publisher_ptr_->msg_.wrench.torque.z = virtual_ft_(5);
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
LBRVirtualFTBroadcaster::damped_least_squares_(const MatT &mat,
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

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRVirtualFTBroadcaster,
                       controller_interface::ControllerInterface)
