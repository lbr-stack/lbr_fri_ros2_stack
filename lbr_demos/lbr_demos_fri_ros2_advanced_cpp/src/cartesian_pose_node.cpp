#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friClientIf.h"
#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

class CartesianPoseNode : public rclcpp::Node {
protected:
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr lbr_position_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_sub_;

  rclcpp::SyncParametersClient::SharedPtr parameter_client_;

  lbr_fri_msgs::msg::LBRState current_robot_state_;     // robot state, including joint positions
  geometry_msgs::msg::Pose current_cartesian_position_; // current cartesian pose of robot

  KDL::Chain chain_; // robot kinematics chain exetracted from robot URDF file

private:
  /**
   * @function: callback function for Joint Position Subscriber
   * @param msg joint position of the robot
   */
  void on_lbr_state_(const lbr_fri_msgs::msg::LBRState &msg) {
    current_robot_state_ = msg;

    double joint_position[7];
    for (int i = 0; i < 7; i++) {
      joint_position[i] = current_robot_state_.measured_joint_position[i];
    }
    current_cartesian_position_ = compute_fk_(joint_position);
    cartesian_pose_pub_->publish(current_cartesian_position_);

    return;
  }

  /**
   * @function: callback function for Cartesian Pose Subscriber
   * @param msg cartesian pose command
   */
  void on_cartesian_pose_(const geometry_msgs::msg::Pose &msg) {
    if (current_robot_state_.session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      unsigned int joint_number = chain_.getNrOfJoints(); // for kuka robot, 7 joints
      KDL::JntArray current_joint_positions(joint_number);
      lbr_fri_msgs::msg::LBRPositionCommand joint_position_command;

      for (unsigned int i = 0; i < joint_number; i++) {
        current_joint_positions(i) = current_robot_state_.measured_joint_position[i];
      }

      joint_position_command = compute_ik_(msg, current_joint_positions);
      lbr_position_command_pub_->publish(joint_position_command);
    }

    return;
  }

public:
  CartesianPoseNode() : Node("cartesian_pose_node") {
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/lbr/app");

    while (!parameter_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "this node has not connected with Parameter Server Node.");
    }
    std::string robot_description_string =
        parameter_client_->get_parameter<std::string>("robot_description");

    KDL::Tree robot_tree;
    if (!kdl_parser::treeFromString(robot_description_string, robot_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree.");
      return;
    }

    std::string root_link = "link_0"; // adjust with your URDF‘s root link
    std::string tip_link = "link_ee"; // adjust with your URDF‘s tip link
    if (!robot_tree.getChain(root_link, tip_link, chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get chain from tree.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Get chain from tree successfully.");
    }

    lbr_position_command_pub_ = this->create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>(
        "/lbr/command/joint_position", 10);
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state", 10,
        std::bind(&CartesianPoseNode::on_lbr_state_, this, std::placeholders::_1));
    cartesian_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::Pose>("/lbr/state/cartesian_pose", 10);
    cartesian_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/lbr/command/cartesian_pose", 10,
        std::bind(&CartesianPoseNode::on_cartesian_pose_, this, std::placeholders::_1));
  }

  /**
   * @function: calculate forward kinematics of robot
   * @param position_array_ptr store seven joint positions of robot
   * @return cartesian pose of the robot
   */
  geometry_msgs::msg::Pose compute_fk_(double *position_array_ptr) {
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(chain_);

    unsigned int joint_number = chain_.getNrOfJoints();
    KDL::JntArray joint_positions = KDL::JntArray(joint_number);

    for (unsigned int i = 0; i < joint_number; i++) {
      joint_positions(i) = position_array_ptr[i];
    }

    KDL::Frame cartesian_pose_temp;          // Cartesian Pose described in data type KDL::Frame
    geometry_msgs::msg::Pose cartesian_pose; // described in geometry_msgs::msg::Pose

    if (fk_solver.JntToCart(joint_positions, cartesian_pose_temp) < 0) {
      RCLCPP_ERROR(this->get_logger(), "FK Solver to calculate JointToCartesian failed.");
    } else {
      // Position
      cartesian_pose.position.x = cartesian_pose_temp.p.x();
      cartesian_pose.position.y = cartesian_pose_temp.p.y();
      cartesian_pose.position.z = cartesian_pose_temp.p.z();

      // Orientation
      double x, y, z, w;
      cartesian_pose_temp.M.GetQuaternion(x, y, z, w); // get quaternion
      cartesian_pose.orientation.x = x;
      cartesian_pose.orientation.y = y;
      cartesian_pose.orientation.z = z;
      cartesian_pose.orientation.w = w;
    }

    return cartesian_pose;
  }

  /**
   * @function: calculate inverse kinematics of robot
   * @param desired_cartesian_pose target cartesian pose we want to transform to joint space
   * @param current_joint_positions current joint positions
   * @return joint positions command
   */
  lbr_fri_msgs::msg::LBRPositionCommand
  compute_ik_(const geometry_msgs::msg::Pose &desired_cartesian_pose,
              KDL::JntArray &current_joint_positions) {
    KDL::ChainIkSolverPos_LMA ik_solver(chain_);
    KDL::JntArray result_joint_positions = KDL::JntArray(chain_.getNrOfJoints());
    lbr_fri_msgs::msg::LBRPositionCommand joint_position_command;

    // transfer data type 'geometry::msg::Pose' to be 'KDL::Frame'
    KDL::Vector position(desired_cartesian_pose.position.x, desired_cartesian_pose.position.y,
                         desired_cartesian_pose.position.z);
    KDL::Rotation rotation = KDL::Rotation::Quaternion(
        desired_cartesian_pose.orientation.x, desired_cartesian_pose.orientation.y,
        desired_cartesian_pose.orientation.z, desired_cartesian_pose.orientation.w);
    KDL::Frame desired_cartesian_pose_temp(rotation, position);

    auto start = std::chrono::high_resolution_clock::now();
    int ik_result = ik_solver.CartToJnt(current_joint_positions, desired_cartesian_pose_temp,
                                        result_joint_positions);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;
    RCLCPP_DEBUG(this->get_logger(), "IK solver execution time: %d ms", execution_time.count());

    if (ik_result < 0) // if solving failed, 'ik_result' would be less than 0
    {
      RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed.");
    } else {
      for (unsigned int i = 0; i < result_joint_positions.data.size(); i++) {
        joint_position_command.joint_position[i] = result_joint_positions(i);
      }
    }

    return joint_position_command;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CartesianPoseNode>());

  rclcpp::shutdown();
  return 0;
}
