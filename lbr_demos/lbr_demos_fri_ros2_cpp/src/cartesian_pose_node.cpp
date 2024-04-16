#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem> // for getting current working directory
#include "geometry_msgs/msg/pose.hpp" // for describing Cartesian Pose
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp" // for forward kinematics
#include "kdl/chainiksolverpos_lma.hpp" // for inverse kinematics
#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "friClientIf.h"

using std::placeholders::_1;

class CartesianPoseNode:public rclcpp::Node
{
  private:
    rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr joint_position_publisher_;
    rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr joint_position_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_subscriber_;

    lbr_fri_msgs::msg::LBRState current_robot_state_; // robot state, including joint positions
    geometry_msgs::msg::Pose current_cartesian_position_; // current cartesian pose of robot

    KDL::Chain chain_; // robot kinematics chain exetracted from robot URDF file

  private:
    /**
     * @function: callback function for Joint Position Subscriber
     * @param msg joint position of the robot
    */
    void joint_position_sub_callback(const lbr_fri_msgs::msg::LBRState& msg)
    {
      current_robot_state_ = msg;
            
      double joint_position[7];
      for(int i = 0; i < 7; i++)
      {
        joint_position[i] = current_robot_state_.measured_joint_position[i]; 
      }
      current_cartesian_position_ = computeForwardKinematics(joint_position);
      cartesian_pose_publisher_->publish(current_cartesian_position_);

      return;
    }

    /**
     * @function: callback function for Cartesian Pose Subscriber
     * @param msg cartesian pose command
    */
    void cartesian_pose_sub_callback(const geometry_msgs::msg::Pose& msg)
    {
      if(current_robot_state_.session_state == KUKA::FRI::COMMANDING_ACTIVE) 
      {
        unsigned int joint_number = chain_.getNrOfJoints(); // for kuka robot, 7 joints 
        KDL::JntArray current_joint_positions(joint_number);
        lbr_fri_msgs::msg::LBRPositionCommand joint_position_command;

        for(unsigned int i = 0; i < joint_number; i++)
        {
          current_joint_positions(i) = current_robot_state_.measured_joint_position[i];
        }

        joint_position_command = computeInverseKinematics(msg, current_joint_positions);
        joint_position_publisher_->publish(joint_position_command);
      }

      return;
    }

  public:
    CartesianPoseNode():Node("cartesian_pose_node")
    {
      this->declare_parameter<std::string>("robot_name", "iiwa7"); // default name "iiwa7"

      std::string robot_name;
      this->get_parameter("robot_name", robot_name);

      if(robot_name != "iiwa7" && robot_name != "iiwa14" && 
         robot_name != "med7" && robot_name != "med14")
      {
        std::cerr << "Wrong robot name. " << std::endl;
      }

      // get the path of urdf file 
      std::filesystem::path current_working_directory = std::filesystem::current_path();
      std::string urdf_file_path = current_working_directory.string() 
                                   + "/src/lbr_fri_ros2_stack/lbr_description/urdf/" 
                                   + robot_name + "/" 
                                   + robot_name + ".urdf"; // path of your robot urdf file

      std::string robot_description_string = readUrdfFile(urdf_file_path);

      KDL::Tree robot_tree;
      if(!kdl_parser::treeFromString(robot_description_string, robot_tree))
      {
        std::cout << "Failed to construct kdl tree." << std::endl;
      }

      std::string root_link = "link_0"; // adjust with your URDF‘s root link
      std::string tip_link = "link_ee"; // adjust with your URDF‘s tip link
      if(!robot_tree.getChain(root_link, tip_link, chain_))
      {
        std::cerr << "Failed to get chain from tree." << std::endl;
      }
      else
      {
        std::cout << "Get chain from tree successfully." << std::endl;
      }

      joint_position_publisher_ = this->create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>(
          "/lbr/command/joint_position", 10);
      joint_position_subscriber_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
          "/lbr/state", 10, 
          std::bind(&CartesianPoseNode::joint_position_sub_callback, this, _1));
      cartesian_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
          "/lbr/state/cartesian_pose", 10);
      cartesian_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/lbr/command/cartesian_pose", 10, 
          std::bind(&CartesianPoseNode::cartesian_pose_sub_callback, this, _1));
    }

    /**
     * @function: convert URDF file to a string
     * @param urdf_file_path the path of URDF file
     * @return string type of URDF file
    */
    std::string readUrdfFile(const std::string& urdf_file_path)
    {
      std::ifstream file_stream(urdf_file_path);
      if(!file_stream) // if open this file failed, return null string
      {
        std::cerr << "Failed to open file at path:" << urdf_file_path << std::endl;
        return "";
      }

      std::stringstream buffer;
      buffer << file_stream.rdbuf();
      return buffer.str();
    }

    /**
     * @function: calculate forward kinematics of robot
     * @param position_array_ptr store seven joint positions of robot 
     * @return cartesian pose of the robot
    */
    geometry_msgs::msg::Pose computeForwardKinematics(double* position_array_ptr)
    {
      KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(chain_);

      unsigned int joint_number = chain_.getNrOfJoints();
      KDL::JntArray joint_positions = KDL::JntArray(joint_number);

      for(unsigned int i = 0; i < joint_number; i++)
      {
        joint_positions(i) = position_array_ptr[i];
      }

      KDL::Frame cartesian_pose_temp; // Cartesian Pose described in data type KDL::Frame
      geometry_msgs::msg::Pose cartesian_pose; // described in geometry_msgs::msg::Pose

      if(fk_solver.JntToCart(joint_positions, cartesian_pose_temp) < 0)
      {
        std::cerr << "FK Solver to calculate JointToCartesian failed." << std::endl;
      }
      else
      {
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
    lbr_fri_msgs::msg::LBRPositionCommand computeInverseKinematics(
        const geometry_msgs::msg::Pose& desired_cartesian_pose, 
        KDL::JntArray& current_joint_positions)
    {
      KDL::ChainIkSolverPos_LMA ik_solver(chain_);
      KDL::JntArray result_joint_positions = KDL::JntArray(chain_.getNrOfJoints());
      lbr_fri_msgs::msg::LBRPositionCommand joint_position_command;

      // transfer data type 'geometry::msg::Pose' to be 'KDL::Frame'
      KDL::Vector position(desired_cartesian_pose.position.x, 
                           desired_cartesian_pose.position.y, 
                           desired_cartesian_pose.position.z); 
      KDL::Rotation rotation =KDL::Rotation::Quaternion(desired_cartesian_pose.orientation.x,
                                                        desired_cartesian_pose.orientation.y,
                                                        desired_cartesian_pose.orientation.z,
                                                        desired_cartesian_pose.orientation.w);
      KDL::Frame desired_cartesian_pose_temp(rotation, position); 

      //auto start = std::chrono::high_resolution_clock::now();
      int ik_result = ik_solver.CartToJnt(current_joint_positions, 
                                          desired_cartesian_pose_temp, 
                                          result_joint_positions); 
      //auto end = std::chrono::high_resolution_clock::now();
      //std::chrono::duration<double, std::milli> execution_time = end - start;
      //std::cout << "IK solver execution time: "<< execution_time.count()<< "ms"<<std::endl;

      if(ik_result < 0) // if solving failed, 'ik_result' would be less than 0
      {
        std::cerr << "Inverse kinematics failed." << std::endl;
      }
      else 
      {
        //std::cout << "Inverse kinematics solution:" << std::endl;
        for(unsigned int i = 0; i < result_joint_positions.data.size(); i++) 
        {
          joint_position_command.joint_position[i] = result_joint_positions(i);
          //std::cout << "Joint " << i << ": "<<result_joint_positions(i) << std::endl;
        }
      }

      return joint_position_command;
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
	
  rclcpp::spin(std::make_shared<CartesianPoseNode>()); 

  rclcpp::shutdown(); 
  return 0;
}
