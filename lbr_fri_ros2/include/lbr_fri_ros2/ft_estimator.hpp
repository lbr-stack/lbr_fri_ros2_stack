#ifndef LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
#define LBR_FRI_ROS2__FT_ESTIMATOR_HPP_

#include <array>
#include <string>

#include "eigen3/Eigen/Core"
#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/pinv.hpp"

#include "friLBRState.h"

namespace lbr_fri_ros2 {
class FTEstimator {
protected:
  static constexpr uint8_t CARTESIAN_DOF = 6;
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::FTEstimator";

public:
  FTEstimator(const std::string &robot_description, const std::string &chain_root = "link_0",
              const std::string &chain_tip = "link_ee");

  void
  compute(const lbr_fri_msgs::msg::LBRState::_measured_joint_position_type &measured_joint_position,
          const lbr_fri_msgs::msg::LBRState::_external_torque_type &external_torque,
          std::array<double, CARTESIAN_DOF> &f_ext);

protected:
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  KDL::Jacobian jacobian_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, CARTESIAN_DOF> jacobian_inv_;
  KDL::JntArray q_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 1> tau_ext_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> f_ext_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FT_ESTIMATOR_HPP_
