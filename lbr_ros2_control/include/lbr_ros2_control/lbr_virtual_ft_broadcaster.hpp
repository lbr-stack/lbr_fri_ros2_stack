#ifndef LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_
#define LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinematics_interface_kdl/kinematics_interface_kdl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "friClientIf.h"
#include "friLBRState.h"

#include "lbr_ros2_control/lbr_system_interface_type_values.hpp"

namespace lbr_ros2_control {
class LBRVirtualFTBroadcaster : public controller_interface::ControllerInterface {
public:
  LBRVirtualFTBroadcaster();

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
  void init_states_();
  bool reference_state_interfaces_();
  void clear_state_interfaces_();

  template <class MatT>
  Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  damped_least_squares_(const MatT &mat,
                        typename MatT::Scalar lambda = typename MatT::Scalar{2e-1});

  kinematics_interface_kdl::KinematicsInterfaceKDL kinematics_interface_kdl_;

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_pinv_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1> joint_positions_,
      external_joint_torques_;
  Eigen::Matrix<double, 6, 1> virtual_ft_;

  double damping_;

  std::string end_effector_link_ = "link_ee";
  std::array<std::string, 7> joint_names_ = {"A1", "A2", "A3", "A4", "A5", "A6", "A7"};

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_interfaces_, external_joint_torque_interfaces_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_publisher_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      rt_wrench_stamped_publisher_ptr_;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_VIRTUAL_FT_BROADCASTER_HPP_
