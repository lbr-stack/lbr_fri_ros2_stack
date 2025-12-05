#include "lbr_fri_ros2/wrench_estimator.hpp"

namespace lbr_fri_ros2 {
WrenchEstimator::WrenchEstimator(const std::string &robot_description,
                                 const WrenchEstimatorParameters &parameters) {
  if (!parameters.valid()) {
    throw std::invalid_argument("Invalid wrench estimator parameters.");
  }
  f_ext_th_[0] = parameters.force_x_th;
  f_ext_th_[1] = parameters.force_y_th;
  f_ext_th_[2] = parameters.force_z_th;
  f_ext_th_[3] = parameters.torque_x_th;
  f_ext_th_[4] = parameters.torque_y_th;
  f_ext_th_[5] = parameters.torque_z_th;
  damping_ = parameters.damping;
  kinematics_ptr_ =
      std::make_unique<Kinematics>(robot_description, parameters.chain_root, parameters.chain_tip);
  reset();
}

void WrenchEstimator::compute() {
  auto jacobian = kinematics_ptr_->compute_jacobian(q_);
  jacobian_inv_ = pinv(jacobian.data, damping_);
  f_ext_raw_ = jacobian_inv_.transpose() * tau_ext_;
  int i = -1;
  f_ext_ = f_ext_raw_.unaryExpr([&](double v) {
    ++i;
    if (std::abs(v) < f_ext_th_[i]) {
      return 0.;
    } else {
      return std::copysign(1., v) * (std::abs(v) - f_ext_th_[i]);
    }
  });

  // rotate into chain tip frame
  auto chain_tip_frame = kinematics_ptr_->compute_fk(q_);
  f_ext_tf_.topRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.topRows(3);
  f_ext_tf_.bottomRows(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data) * f_ext_.bottomRows(3);
}

void WrenchEstimator::reset() {
  std::fill(q_.begin(), q_.end(), 0.0);
  tau_ext_.setZero();
  f_ext_raw_.setZero();
  f_ext_.setZero();
  f_ext_tf_.setZero();
  jacobian_inv_.setZero();
}

void WrenchEstimator::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Damping: %.3f", damping_);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Force thresholds: [%.3f, %.3f, %.3f]",
              f_ext_th_[0], f_ext_th_[1], f_ext_th_[2]);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   Torque thresholds: [%.3f, %.3f, %.3f]",
              f_ext_th_[3], f_ext_th_[4], f_ext_th_[5]);
}
} // namespace lbr_fri_ros2
