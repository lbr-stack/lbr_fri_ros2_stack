#ifndef LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_
#define LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_

#include <cstring>
#include <string>

#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

#include "damped_least_squares.hpp"

class AdmittanceController {
  using JointVector = Eigen::Vector<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using CartesianVector = Eigen::Vector<double, 6>;

public:
  AdmittanceController(const std::string &robot_description,
                       const std::string &base_link = "lbr_link_0",
                       const std::string &end_effector_link = "lbr_link_ee",
                       const CartesianVector &f_ext_th = {2., 2., 2., 0.5, 0.5, 0.5},
                       const JointVector &dq_gains = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5},
                       const CartesianVector &dx_gains = {1.5, 1.5, 1.5, 20., 40., 60.})
      : dq_gains_(dq_gains), dx_gains_(dx_gains), f_ext_th_(f_ext_th) {
    if (!kdl_parser::treeFromString(robot_description, tree_)) {
      throw std::runtime_error("Failed to construct kdl tree from robot description.");
    }
    if (!tree_.getChain(base_link, end_effector_link, chain_)) {
      throw std::runtime_error("Failed to construct kdl chain from robot description.");
    }

    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    jacobian_.resize(chain_.getNrOfJoints());
    q_.resize(chain_.getNrOfJoints());
  };

  const lbr_fri_msgs::msg::LBRCommand &update(const lbr_fri_msgs::msg::LBRState &lbr_state) {
    std::memcpy(q_.data.data(), lbr_state.measured_joint_position.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    std::memcpy(tau_ext_.data(), lbr_state.external_torque.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    jacobian_solver_->JntToJac(q_, jacobian_);

    jacobian_inv_ = damped_least_squares(jacobian_.data);
    f_ext_ = jacobian_inv_.transpose() * tau_ext_;

    for (int i = 0; i < 6; i++) {
      if (std::abs(f_ext_[i]) < f_ext_th_[i]) {
        f_ext_[i] = 0.;
      } else {
        f_ext_[i] = std::copysign(dx_gains_[i], f_ext_[i]) * (std::abs(f_ext_[i]) - f_ext_th_[i]);
      }
    }

    dq_ = dq_gains_.asDiagonal() * jacobian_inv_ * f_ext_;

    for (int i = 0; i < 7; i++) {
      lbr_command_.joint_position[i] =
          lbr_state.measured_joint_position[i] + dq_[i] * lbr_state.sample_time;
    }

    return lbr_command_;
  };

protected:
  lbr_fri_msgs::msg::LBRCommand lbr_command_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  KDL::Jacobian jacobian_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 6> jacobian_inv_;
  KDL::JntArray q_;
  JointVector dq_;
  JointVector tau_ext_;
  JointVector dq_gains_;
  CartesianVector dx_gains_;
  CartesianVector f_ext_;
  CartesianVector f_ext_th_;
};
#endif // LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_
