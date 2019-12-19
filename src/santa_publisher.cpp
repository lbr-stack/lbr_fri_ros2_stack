#include <chrono>
#include <math.h>
#include <tuple>
#include <vector>
#include <cmath>
#include <fstream>
#include <cstring>

#define DEG2RAD M_PI/180.

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// FRI
#include <fri/friLBRState.h>
#include <fri/friLBRClient.h>

// custom
#include <ramp_function.h>

using namespace std::chrono_literals;

auto PTP(double* q_start, const double* q_end, double dt = 0.005, double a = .1, double vmax = .1) -> std::vector<std::vector<double>> {

	double dq[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; // defaults zero

	for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
		dq[i] = q_end[i] - q_start[i];
	}

	// find longest for scaling ptp motion
	double longest = 0.;
	for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
		if (std::abs(dq[i]) > longest) {
			longest = std::abs(dq[i]);
		}
	}


	RampFunction rf(longest, a, vmax);

	double tend = rf.getEndTime();
	std::vector<double> q(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.);
	std::vector<std::vector<double>> trajectory;


	for (double t = 0.; t < tend; t+=dt) {
		for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
			q[i] = q_start[i] + dq[i]/longest*rf.s(t);
		}
		trajectory.push_back(q);
	}

	return trajectory;
}

std::vector<std::vector<double>> read_from_file(std::fstream& in) {
    std::vector<std::vector<double>> table;
    std::vector<double> row;

    std::string line, element;

    while (std::getline(in, line)) {
        row.clear();
        std::stringstream s(line);

        while (std::getline(s, element, ',')) {
            row.push_back(std::stod(element)*DEG2RAD);
        }
        table.push_back(row);
    }

    return table;
}

int counter = 0;
	
class Publisher : public rclcpp::Node {

    public:
        Publisher() 
            : Node("fri_test_publisher"),
              dt_(0.0) {
            // timer_ = this->create_wall_timer(5ms, std::bind(&Publisher::timer_callback, this));   
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lbr_joint_angles_in", 10);
            sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/lbr_joint_angles_out", 10, std::bind(&Publisher::callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "opening file");
            // open file to read in joint angle commands from
            in_file_.open("/home/maritn/Documents/dev_ws/src/fast_robot_interface_ros2/vscode/trajectory.csv", std::ios::in);
            traj_ = read_from_file(in_file_);
            RCLCPP_INFO(this->get_logger(), "initial position is (%f, %f, %f, %f, %f, %f, %f)",
                traj_[0][0],
                traj_[0][1],
                traj_[0][2],
                traj_[0][3],
                traj_[0][4],
                traj_[0][5],
                traj_[0][6]
            );
        }

        ~Publisher() {
            in_file_.close();
        }

    private:
        void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
            // read current position and perform ptp motion to initial position until close to initial
            std::vector<double> out(KUKA::FRI::LBRState::NUMBER_OF_JOINTS); // angles read from lbr, thus out

            std::memcpy(out.data(), msg->data.data(), sizeof(double)*KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

            double epsilon = 0.1;
            bool close = false;

            RCLCPP_INFO(this->get_logger(), "current robot state is (%f, %f, %f, %f, %f, %f, %f)",
                out[0],
                out[1],
                out[2],
                out[3],
                out[4],
                out[5],
                out[6]
            );

            if ( std::abs( traj_[0][0] - out[0]) > epsilon || 
                 std::abs( traj_[0][1] - out[1]) > epsilon ||
                 std::abs( traj_[0][2] - out[2]) > epsilon ||
                 std::abs(-traj_[0][3] - out[3]) > epsilon ||
                 std::abs( traj_[0][4] - out[4]) > epsilon ||
                 std::abs( traj_[0][5] - out[5]) > epsilon ||
                 std::abs( traj_[0][6] - out[6]) > epsilon) { // perform ptp motion to init
                // compute a point to point motion
                if (out[0] > 3. ||
                    out[1] > 3. ||
                    out[2] > 3. ||
                    out[3] > 3. ||
                    out[4] > 3. ||
                    out[5] > 3. ||
                    out[6] > 3.)
                { // create if statement to publish first to fri node so that udp connection gets established

                    // collect output joint angles
                    std::vector<double> traj(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.);

                    // prepare output joint angles as multiarraz message
                    auto msg = std_msgs::msg::Float64MultiArray();
                    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                    msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
                    msg.layout.dim[0].stride = 1;
                    msg.layout.dim[0].label = "i";

                    msg.data.clear();
                    msg.data.insert(msg.data.end(), traj.begin(), traj.end()); // TODO replace by motion

                    // publish message to fri node where it is written via UDP to the LBR
                    pub_->publish(msg);
                }
                else {
                    auto motion = PTP(out.data(), traj_[0].data(), 0.005);
                    // std::vector<double> motion();

                    // collect output joint angles
                    std::vector<double> traj = motion[0];

                    RCLCPP_INFO(this->get_logger(), "moving to initial position");

                    // prepare output joint angles as multiarraz message
                    auto msg = std_msgs::msg::Float64MultiArray();
                    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                    msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
                    msg.layout.dim[0].stride = 1;
                    msg.layout.dim[0].label = "i";

                    msg.data.clear();
                    msg.data.insert(msg.data.end(), traj.begin(), traj.end()); // TODO replace by motion

                    // publish message to fri node where it is written via UDP to the LBR
                    pub_->publish(msg);
                }
            }       
            else { // read in lin motion from csv file and publish to topic   
                std::vector<double> traj = traj_[counter];
                if (counter < traj_.size()) {
                    counter++;
                }
                RCLCPP_INFO(this->get_logger(), "running linear motion");

                // prepare output joint angles as multiarraz message
                auto msg = std_msgs::msg::Float64MultiArray();
                msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "i";

                msg.data.clear();
                msg.data.insert(msg.data.end(), traj.begin(), traj.end());

                // publish message to fri node where it is written via UDP to the LBR
                pub_->publish(msg);
            } 
        }

        void timer_callback() { // not used
			// collect output joint angles
            std::vector<double> out{0., 0., 0., 0., 0., 0., 0.};
            RCLCPP_INFO(this->get_logger(), "publishing");

			// prepare output joint angles as multiarraz message
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "i";

            msg.data.clear();
            msg.data.insert(msg.data.end(), out.begin(), out.end());

			// publish message to fri node where it is written via UDP to the LBR
            pub_->publish(msg);
        };

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_; // subscribe to joint angles read from fri node, that should rather be a action server

        double dt_;      //!< stepwidth for sine 
        std::fstream in_file_;
        std::vector<std::vector<double>> traj_;
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}