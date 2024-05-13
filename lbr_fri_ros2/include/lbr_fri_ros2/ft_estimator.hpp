#ifndef LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
#define LBR_FRI_ROS2__FT_ESTIMATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "eigen3/Eigen/Core"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/pinv.hpp"

namespace lbr_fri_ros2 {
class FTEstimator {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::FTEstimator";
  using jnt_pos_array_t = lbr_fri_idl::msg::LBRState::_measured_joint_position_type;
  using const_jnt_pos_array_t_ref = const jnt_pos_array_t &;
  using ext_tau_array_t = lbr_fri_idl::msg::LBRState::_external_torque_type;
  using const_ext_tau_array_t_ref = const ext_tau_array_t &;

public:
  static constexpr uint8_t CARTESIAN_DOF = 6;
  using cart_array_t = std::array<double, CARTESIAN_DOF>;
  using cart_array_t_ref = cart_array_t &;
  using const_cart_array_t_ref = const cart_array_t &;

  FTEstimator(const std::string &robot_description, const std::string &chain_root = "link_0",
              const std::string &chain_tip = "link_ee",
              const_cart_array_t_ref f_ext_th = {2., 2., 2., 0.5, 0.5, 0.5});
  void compute(const_jnt_pos_array_t_ref measured_joint_position,
               const_ext_tau_array_t_ref external_torque, cart_array_t_ref f_ext,
               const double &damping = 0.2);
  void reset();

protected:
  // force threshold
  cart_array_t f_ext_th_;

  KDL::Tree tree_;
  KDL::Chain chain_;

  // solvers
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // robot state
  KDL::JntArray q_;

  // forward kinematics
  KDL::Frame chain_tip_frame_;

  // force estimation
  KDL::Jacobian jacobian_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1> tau_ext_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> f_ext_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
