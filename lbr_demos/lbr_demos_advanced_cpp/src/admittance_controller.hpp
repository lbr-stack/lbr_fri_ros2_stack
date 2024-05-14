#ifndef LBR_DEMOS_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_
#define LBR_DEMOS_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_

#include <cstring>
#include <string>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_joint_position_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/pinv.hpp"

namespace lbr_demos {
class AdmittanceController {
  using joint_vector_t = Eigen::Vector<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using cartesian_vector_t = Eigen::Vector<double, 6>;

public:
  AdmittanceController(const std::string &robot_description,
                       const std::string &base_link = "link_0",
                       const std::string &end_effector_link = "link_ee",
                       const std::vector<double> &f_ext_th = {2., 2., 2., 0.5, 0.5, 0.5},
                       const std::vector<double> &dq_gains = {2., 2., 2., 2., 2., 2., 2.},
                       const std::vector<double> &dx_gains = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1})
      : dq_gains_(dq_gains.data()), dx_gains_(dx_gains.data()), f_ext_th_(f_ext_th.data()) {
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

  const lbr_fri_idl::msg::LBRJointPositionCommand &
  update(const lbr_fri_idl::msg::LBRState &lbr_state, const double &dt) {
    std::memcpy(q_.data.data(), lbr_state.measured_joint_position.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    std::memcpy(tau_ext_.data(), lbr_state.external_torque.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    jacobian_solver_->JntToJac(q_, jacobian_);

    jacobian_inv_ = lbr_fri_ros2::pinv(jacobian_.data);
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
      lbr_joint_position_command_.joint_position[i] =
          lbr_state.measured_joint_position[i] + dq_[i] * dt;
    }

    return lbr_joint_position_command_;
  };

protected:
  lbr_fri_idl::msg::LBRJointPositionCommand lbr_joint_position_command_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  KDL::Jacobian jacobian_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 6> jacobian_inv_;
  KDL::JntArray q_;
  joint_vector_t dq_;
  joint_vector_t tau_ext_;
  joint_vector_t dq_gains_;
  cartesian_vector_t dx_gains_;
  cartesian_vector_t f_ext_;
  cartesian_vector_t f_ext_th_;
};
} // end of namespace lbr_demos
#endif // LBR_DEMOS_ADVANCED_CPP__ADMITTANCE_CONTROLLER_HPP_
