#include <algorithm>

#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr_app.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

// damped least squares solutions:
// http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
dampedLeastSquares(const MatT &mat, typename MatT::Scalar lambda =
                                        typename MatT::Scalar{2e-1}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    dampedSingularValuesInv(i, i) =
        singularValues(i) / (singularValues(i) * singularValues(i) + lambda * lambda);
  }
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
}

class AdmittanceController {
public:
  AdmittanceController(const std::string &robot_description, const std::string &base_link,
                       const std::string &end_effector_link) {
    if (!kdl_parser::treeFromString(robot_description, tree_)) {
      throw std::runtime_error("Failed to construct kdl tree from robot description");
    }
    if (!tree_.getChain(base_link, end_effector_link, chain_)) {
      throw std::runtime_error("Failed to construct kdl chain from robot description");
    }
    jac_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    jacobian_.resize(chain_.getNrOfJoints());
    q_.resize(chain_.getNrOfJoints());
  };

  const lbr_fri_msgs::msg::LBRCommand &update(const lbr_fri_msgs::msg::LBRState &lbr_state) {
    std::copy(lbr_state.measured_joint_position.begin(), lbr_state.measured_joint_position.end(),
              q_.data.data());
    std::copy(lbr_state.external_torque.begin(), lbr_state.external_torque.end(), tau_ext_.data());

    jac_->JntToJac(q_, jacobian_);

    auto jacobian_inv = dampedLeastSquares(jacobian_.data);
    f_ext_ = jacobian_inv.transpose() * tau_ext_;

    for (int i = 0; i < 6; i++) {
      if (std::abs(f_ext_[i]) < f_ext_th_[i]) {
        f_ext_[i] = 0.;
      } else {
        f_ext_[i] -= f_ext_th_[i] * std::copysign(1., f_ext_[i]);
      }
    }

    dq_ = jacobian_inv * f_ext_ * 0.1;

    // double alpha = 0.99;
    for (int i = 0; i < 7; i++) {
      lbr_command_.joint_position[i] =
          lbr_state.measured_joint_position[i] + dq_[i] * lbr_state.sample_time * 30.;
    }

    std::cout << "f_ext: " << f_ext_.transpose() << std::endl;
    std::cout << "tau_ext: " << tau_ext_.transpose() << std::endl;
    std::cout << "dq: " << dq_.transpose() * lbr_state.sample_time << std::endl;

    return lbr_command_;
  };

protected:
  lbr_fri_msgs::msg::LBRCommand lbr_command_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
  KDL::Jacobian jacobian_;
  KDL::JntArray q_;
  Eigen::Vector<double, 7> dq_;
  Eigen::Vector<double, 7> tau_ext_;
  Eigen::Vector<double, 6> f_ext_;
  Eigen::Vector<double, 6> f_ext_th_{2., 2., 2., 0.5, 0.5, 0.5};
};

class AdmittanceControlNode : public rclcpp::Node {
public:
  AdmittanceControlNode(const std::string &node_name, const rclcpp::NodeOptions &options)
      : rclcpp::Node(node_name, options) {
    this->declare_parameter<std::string>("robot_description");
    this->declare_parameter<std::string>("base_link", "lbr_link_0");
    this->declare_parameter<std::string>("end_effector_link", "lbr_link_ee");

    admittance_controller_ = std::make_unique<AdmittanceController>(
        this->get_parameter("robot_description").as_string(),
        this->get_parameter("base_link").as_string(),
        this->get_parameter("end_effector_link").as_string());

    lbr_command_pub_ =
        create_publisher<lbr_fri_msgs::msg::LBRCommand>("/lbr_command", rclcpp::SensorDataQoS());
    lbr_state_sub_ = create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SensorDataQoS(),
        std::bind(&AdmittanceControlNode::on_lbr_state, this, std::placeholders::_1));
  }

protected:
  void on_lbr_state(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    if (!lbr_state) {
      return;
    }

    smoothing(lbr_state, 0.99, smooth_lbr_state_, init_);

    auto lbr_command = admittance_controller_->update(smooth_lbr_state_);
    lbr_command_pub_->publish(lbr_command);
  };

  void smoothing(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state, double alpha,
                 lbr_fri_msgs::msg::LBRState &smooth_lbr_state, bool &init) {
    if (!init) {
      smooth_lbr_state = *lbr_state;
      init = true;
      return;
    }

    for (int i = 0; i < 7; i++) {
      smooth_lbr_state.measured_joint_position[i] =
          lbr_state->measured_joint_position[i] * (1 - alpha) +
          smooth_lbr_state.measured_joint_position[i] * alpha;
      smooth_lbr_state.external_torque[i] =
          lbr_state->external_torque[i] * (1 - alpha) + smooth_lbr_state.external_torque[i] * alpha;
    }
  }

  lbr_fri_msgs::msg::LBRState smooth_lbr_state_;

  bool init_{false};

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;

  std::unique_ptr<AdmittanceController> admittance_controller_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto lbr_app_node = std::make_shared<rclcpp::Node>(
      "lbr_app", rclcpp::NodeOptions().use_intra_process_comms(true));
  auto lbr_app = lbr_fri_ros2::LBRApp(lbr_app_node);
  auto admittance_control_node = std::make_shared<AdmittanceControlNode>(
      "admittance_control_node", rclcpp::NodeOptions().use_intra_process_comms(true));

  executor->add_node(lbr_app_node);
  executor->add_node(admittance_control_node);
  executor->spin();

  return 0;
}