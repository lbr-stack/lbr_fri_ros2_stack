#include "lbr_fri_ros2/ft_estimator.hpp"

namespace lbr_fri_ros2 {
FTEstimator::FTEstimator(const std::string &robot_description, const std::string &chain_root,
                         const std::string &chain_tip) {
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
  jacobian_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  q_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
}

void FTEstimator::compute(
    const lbr_fri_msgs::msg::LBRState::_measured_joint_position_type &measured_joint_position,
    const lbr_fri_msgs::msg::LBRState::_external_torque_type &external_torque,
    std::array<double, CARTESIAN_DOF> &f_ext) {
  q_.data = Eigen::Map<const Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>>(
      measured_joint_position.data());
  tau_ext_ = Eigen::Map<const Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>>(
      external_torque.data());
  jacobian_solver_->JntToJac(q_, jacobian_);
  jacobian_inv_ = pinv(jacobian_.data);
  f_ext_ = jacobian_inv_.transpose() * tau_ext_;
  Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_;
}
} // end of namespace lbr_fri_ros2
