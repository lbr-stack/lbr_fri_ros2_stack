#include "rclcpp/rclcpp.hpp"
#include "cmath"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "chrono"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp" // for forward kinematics
#include "kdl/chainiksolverpos_lma.hpp" // for inverse kinematics
#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "friClientIf.h"


using namespace std::chrono_literals; // this namespace to enable we can use 500ms to represent 500 milliseconds in our code.
using std::placeholders::_1;

lbr_fri_msgs::msg::LBRState current_robot_state; // store current robot state, including measured joint positions
KDL::Frame initial_cartesian_position; // we want to move the end effector in sine curve in z-direction in Cartesian space, this is the initial position 

class JointPositionPublisher:public rclcpp::Node
{
	private:
		rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr publisher_;
        rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr subscriber_;

        KDL::Chain chain_; // robot kinematics chain exetracted from robot URDF file

        double amplitude_; // rad
        double frequency_; // Hz
        double sampling_time_; // sampling time for the state of robot and sending position command
        double phase_; // initial phase
		int count_;

	private:
        /**
         * @function: callback function for Robot State Subscriber
        */
        void topic_callback(const lbr_fri_msgs::msg::LBRState& msg)
        {
            current_robot_state = msg;
            
            if(count_ < 200) // in first 2 seconds, to make 'initial_position_command' be 'measured_joint_position'
            {
                std::cout << "count_" << count_ << std::endl;
                double joint_position[7];
                for(int i = 0; i < 7; i++)
                {
                    joint_position[i] = current_robot_state.measured_joint_position[i]; 
                }
                initial_cartesian_position = computeForwardKinematics(chain_, joint_position);
            }
            else
            {
                if (current_robot_state.session_state == KUKA::FRI::COMMANDING_ACTIVE) 
                {
                    KDL::Frame cartesian_position_command = initial_cartesian_position;

                    cartesian_position_command.p = KDL::Vector(initial_cartesian_position.p.x(), initial_cartesian_position.p.y(), initial_cartesian_position.p.z() + amplitude_ * sin(phase_));

                    phase_ = phase_ + 2 * M_PI * frequency_ * sampling_time_;

                    unsigned int joint_number = chain_.getNrOfJoints(); // for kuka iiwa7 robot, this will be 7 
                    KDL::JntArray current_joint_positions(joint_number);

                    for(unsigned int i = 0; i < joint_number; i++)
                    {
                        current_joint_positions(i) = current_robot_state.measured_joint_position[i];
                    }

                    lbr_fri_msgs::msg::LBRPositionCommand joint_position_command = computeInverseKinematics(chain_, cartesian_position_command, current_joint_positions);

                    publisher_->publish(joint_position_command);
                }
            }

            count_++;

            return;
        }

	public:
		JointPositionPublisher():Node("joint_position_publisher_node"), amplitude_(0.05), frequency_(0.5), sampling_time_(0.01)
		{
            phase_ = 0.0; 
            count_= 0.0;

            std::string urdf_file_path = "/home/nearlab-iiwa/lbr-stack/src/lbr_fri_ros2_stack/lbr_description/urdf/iiwa7/my_iiwa7.urdf"; // path of your robot urdf file
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

			publisher_ = this->create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>("/lbr/command/joint_position", 10);
            subscriber_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>("/lbr/state", 10, std::bind(&JointPositionPublisher::topic_callback, this, _1));
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
         * @param chain robot kinematics chain parsed from URDF file
         * @param position_array_ptr seven joint positions of robot store in double position_array_ptr[7]
         * @return no value
        */
        KDL::Frame computeForwardKinematics(KDL::Chain& chain, double* position_array_ptr)
        {
            KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(chain);

            unsigned int joint_number = chain.getNrOfJoints();
            KDL::JntArray joint_positions = KDL::JntArray(joint_number);

            for(unsigned int i = 0; i < joint_number; i++)
            {
                joint_positions(i) = position_array_ptr[i];
            }

            KDL::Frame cartesian_position;

            if(fk_solver.JntToCart(joint_positions, cartesian_position) < 0)
            {
                std::cerr << "FK Solver to calculate JointToCartesian failed." << std::endl;
            }
            else
            {
                double x, y, z, roll, pitch, yaw;
                x = cartesian_position.p.x(); // get position
                y = cartesian_position.p.y();
                z = cartesian_position.p.z();
            
                cartesian_position.M.GetRPY(roll, pitch, yaw); // get RPY value

                //std::cout << "Position: [" << x << ", " << y << ", " << z << "]" << std::endl;
                //std::cout << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
            }

            return cartesian_position;
        }

        /**
         * @function: calculate inverse kinematics of robot
         * @param chain kinematics chain of robot got from URDF file 
         * @param desired_end_effector_pose target cartesian pose we want to transform to joint space
         * @param current_joint_positions current joint positions
         * @return joint positions command 
        */
        lbr_fri_msgs::msg::LBRPositionCommand computeInverseKinematics(KDL::Chain& chain, KDL::Frame& desired_end_effector_pose, KDL::JntArray& current_joint_positions)
        {
            KDL::ChainIkSolverPos_LMA ik_solver(chain);
            KDL::JntArray result_joint_positions = KDL::JntArray(chain.getNrOfJoints());
            lbr_fri_msgs::msg::LBRPositionCommand joint_position_command;

            auto start = std::chrono::high_resolution_clock::now();
            int ik_result = ik_solver.CartToJnt(current_joint_positions, desired_end_effector_pose, result_joint_positions); // if solving failed, return value would be less than 0
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> execution_time = end - start;
            std::cout << "IK solver execution time: " << execution_time.count() << "ms" << std::endl;

            if (ik_result < 0)
            {
                std::cerr << "Inverse kinematics failed." << std::endl;
            }
            else 
            {
                //std::cout << "Inverse kinematics solution:" << std::endl;
                for (unsigned int i = 0; i < result_joint_positions.data.size(); i++) 
                {
                    joint_position_command.joint_position[i] = result_joint_positions(i);
                    //std::cout << "Joint " << i << ": " << result_joint_positions(i) << std::endl;
                }
            }

            return joint_position_command;
        }
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<JointPositionPublisher>()); 

	rclcpp::shutdown(); 
	return 0;
}