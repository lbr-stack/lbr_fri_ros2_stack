#ifndef LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_components/force_torque_sensor.hpp"

#include "friLBRState.h"
#include "lbr_fri_ros2/control.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/math.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class AdmittanceController : public controller_interface::ControllerInterface {
public:
  AdmittanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool assign_state_interfaces_();
  void release_state_interfaces_();
  void configure_joint_names_();
  void configure_admittance_impl_();
  void configure_inv_jac_ctrl_impl_();
  void configure_filters_();
  void configure_safety_checks_();
  void zero_all_values_();
  void init_filters_with_update_rate_();
  bool any_external_force_torques_on_horizon_(
      const double &max_external_force = 0., const double &max_external_torque = 0.,
      const std::chrono::milliseconds &horizon = std::chrono::milliseconds(200)) const;
  void log_info_() const;

  // safety checks
  double max_external_force_on_activate_{0.};
  double max_external_torque_on_activate_{0.};

  // admittance
  bool initialized_ = false;
  std::unique_ptr<lbr_fri_ros2::AdmittanceImpl> admittance_impl_ptr_;
  Eigen::Matrix<double, 3, 1> t_init_, t_, t_prev_; // translation
  Eigen::Quaterniond r_init_, r_, r_prev_;          // rotation
  Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> f_ext_, f_ext_filtered_, delta_x_, dx_,
      ddx_;

  // external force smoothing
  std::unique_ptr<lbr_fri_ros2::ExponentialFilterArray<lbr_fri_ros2::CARTESIAN_DOF>>
      f_ext_filter_ptr_;

  // joint veloctiy computation
  std::unique_ptr<lbr_fri_ros2::InvJacCtrlImpl> inv_jac_ctrl_impl_ptr_;
  lbr_fri_ros2::jnt_array_t q_, dq_;
  Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> twist_command_;

  // interfaces
  lbr_fri_ros2::jnt_name_array_t joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_;
  std::unique_ptr<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      session_state_interface_ptr_;
  std::string ft_sensor_name_;
  std::unique_ptr<semantic_components::ForceTorqueSensor> estimated_ft_sensor_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
