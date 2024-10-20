#include "lbr_fri_ros2/ft_estimator.hpp"

namespace lbr_fri_ros2 {
FTEstimator::FTEstimator(const std::string &robot_description, const std::string &chain_root,
                         const std::string &chain_tip, const_cart_array_t_ref f_ext_th)
    : f_ext_th_(f_ext_th) {
  kinematics_ = std::make_unique<Kinematics>(robot_description, chain_root, chain_tip);
  reset();
}

void FTEstimator::compute(const_jnt_pos_array_t_ref measured_joint_position,
                          const_ext_tau_array_t_ref external_torque, cart_array_t_ref f_ext,
                          const double &damping) {
  tau_ext_ = Eigen::Map<const Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1>>(
      external_torque.data());
  auto jacobian = kinematics_->compute_jacobian(measured_joint_position);
  jacobian_inv_ = pinv(jacobian.data, damping);
  f_ext_ = jacobian_inv_.transpose() * tau_ext_;

  // rotate into chain tip frame
  auto chain_tip_frame = kinematics_->compute_fk(measured_joint_position);
  f_ext_.topRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.topRows(3);
  f_ext_.bottomRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.bottomRows(3);

  Eigen::Map<Eigen::Matrix<double, Kinematics::CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_;

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
  tau_ext_.setZero();
  f_ext_.setZero();
}
} // namespace lbr_fri_ros2
