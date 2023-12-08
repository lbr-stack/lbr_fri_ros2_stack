#include "lbr_fri_ros2/ft_estimator.hpp"

namespace lbr_fri_ros2 {
FTEstimator::FTEstimator(const std::string &robot_description, const std::string &chain_root,
                         const std::string &chain_tip, const_cart_array_t_ref f_ext_th)
    : f_ext_th_(f_ext_th) {
  if (!kdl_parser::treeFromString(robot_description, tree_)) {
    std::string err = "Failed to construct kdl tree from robot description.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  if (!tree_.getChain(chain_root, chain_tip, chain_)) {
    std::string err = "Failed to construct kdl chain from robot description.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), err.c_str());
    throw std::runtime_error(err);
  }
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  q_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  reset();
}

void FTEstimator::compute(const_jnt_pos_array_t_ref measured_joint_position,
                          const_ext_tau_array_t_ref external_torque, cart_array_t_ref f_ext,
                          const double &damping) {
  q_.data = Eigen::Map<const Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>>(
      measured_joint_position.data());
  tau_ext_ = Eigen::Map<const Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>>(
      external_torque.data());
  jacobian_solver_->JntToJac(q_, jacobian_);
  jacobian_inv_ = pinv(jacobian_.data, damping);
  f_ext_ = jacobian_inv_.transpose() * tau_ext_;

  // rotate into chain tip frame
  fk_solver_->JntToCart(q_, chain_tip_frame_);
  f_ext_.topRows(3) = Eigen::Matrix3d::Map(chain_tip_frame_.M.data) * f_ext_.topRows(3);
  f_ext_.bottomRows(3) = Eigen::Matrix3d::Map(chain_tip_frame_.M.data) * f_ext_.bottomRows(3);

  Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_;

  // threshold force-torque
  std::transform(f_ext.begin(), f_ext.end(), f_ext_th_.begin(), f_ext.begin(),
                 [](const double &f_ext_i, const double &f_ext_th_i) {
                   if (std::abs(f_ext_i) < f_ext_th_i) {
                     return 0.;
                   } else {
                     return std::copysign(1., f_ext_i) * (std::abs(f_ext_i) - f_ext_th_i);
                   }
                 });
}

void FTEstimator::reset() {
  q_.data.setZero();
  tau_ext_.setZero();
  f_ext_.setZero();
}
} // end of namespace lbr_fri_ros2
