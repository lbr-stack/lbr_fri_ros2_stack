#ifndef LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
#define LBR_FRI_ROS2__FT_ESTIMATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "eigen3/Eigen/Core"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/pinv.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
class FTEstimator {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::FTEstimator";

public:
  FTEstimator(const std::string &robot_description, const std::string &chain_root = "lbr_link_0",
              const std::string &chain_tip = "lbr_link_ee",
              const_cart_array_t_ref f_ext_th = {2., 2., 2., 0.5, 0.5, 0.5});
  void compute(const_jnt_array_t_ref measured_joint_position, const_jnt_array_t_ref external_torque,
               cart_array_t_ref f_ext, const double &damping = 0.2);
  void reset();

protected:
  // force threshold
  cart_array_t f_ext_th_;

  // kinematics
  std::unique_ptr<Kinematics> kinematics_ptr_;

  // force estimation
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1> tau_ext_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> f_ext_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
