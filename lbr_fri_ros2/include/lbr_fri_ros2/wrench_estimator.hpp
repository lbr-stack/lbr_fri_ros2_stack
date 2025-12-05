#ifndef LBR_FRI_ROS2__WRENCH_ESTIMATOR_HPP_
#define LBR_FRI_ROS2__WRENCH_ESTIMATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "eigen3/Eigen/Core"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/pinv.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct WrenchEstimatorParameters {
  std::string chain_root{"lbr_link_0"};
  std::string chain_tip{"lbr_link_ee"};
  double damping{0.2};
  double force_x_th{2.0};
  double force_y_th{2.0};
  double force_z_th{2.0};
  double torque_x_th{0.5};
  double torque_y_th{0.5};
  double torque_z_th{0.5};

  bool valid() const {
    return !(chain_root.empty() || chain_tip.empty() || damping < 0.0 || force_x_th < 0.0 ||
             force_y_th < 0.0 || force_z_th < 0.0 || torque_x_th < 0.0 || torque_y_th < 0.0 ||
             torque_z_th < 0.0);
  }
};

class WrenchEstimator {
  /**
   * @brief A class to estimate wrenches from external joint torque readings. Note that only
   * forces beyond a specified threshold are returned. The specified threshold is removed from the
   * estimated force-torque.
   *
   */
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::WrenchEstimator";

public:
  WrenchEstimator() = delete;
  WrenchEstimator(const std::string &robot_description,
                  const WrenchEstimatorParameters &parameters = WrenchEstimatorParameters());
  void compute();
  void reset();

  inline void get_f_ext(cart_array_t_ref f_ext) const {
    Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_;
  }
  inline void get_f_ext_tf(cart_array_t_ref f_ext) const {
    Eigen::Map<Eigen::Matrix<double, CARTESIAN_DOF, 1>>(f_ext.data()) = f_ext_tf_;
  }
  inline void set_tau_ext(const_jnt_array_t_ref tau_ext) {
    tau_ext_ = Eigen::Map<const Eigen::Matrix<double, N_JNTS, 1>>(tau_ext.data());
  }
  inline void set_q(const_jnt_array_t_ref q) { q_ = q; }

  void log_info() const;

protected:
  // force threshold
  cart_array_t f_ext_th_;

  // damping for pseudo-inverse of Jacobian
  double damping_;

  // joint positions and external joint torques
  jnt_array_t q_;

  // kinematics
  std::unique_ptr<Kinematics> kinematics_ptr_;

  // force estimation
  Eigen::Matrix<double, N_JNTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, N_JNTS, 1> tau_ext_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> f_ext_raw_, f_ext_, f_ext_tf_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__WRENCH_ESTIMATOR_HPP_
