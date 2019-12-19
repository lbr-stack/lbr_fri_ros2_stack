#include <chrono>
#include <math.h>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fri/friLBRState.h>

// custom
#include <ramp_function.h>

#define DEG2RAD M_PI/180.

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

    for (double t = 0.; t < 10.; t+=dt) { // do nothin for 10 seconds
        trajectory.push_back(std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.));
    }

	for (double t = 0.; t < tend; t+=dt) {
		for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
			q[i] = q_start[i] + dq[i]/longest*rf.s(t);
		}
		trajectory.push_back(q);
	}

	return trajectory;
}

// read from csv
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


class PTPPublisher : public rclcpp::Node {

    public:
        PTPPublisher() 
            : Node("fri_test_publisher") {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lbr_joint_angles_in", 10);
            timer_ = this->create_wall_timer(5ms, std::bind(&PTPPublisher::timer_callback, this));
            double q_start[7] = {0., 0., 0., 0., 0., 0., 0.};
            double q_end[7] = {0., 30.*DEG2RAD, 0., 60*DEG2RAD, 0., 30.*DEG2RAD, 0.};
            ptp_ = PTP(q_start, q_end, 0.005); 

            // read in linear motion
            std::fstream in_file;
            in_file.open("/home/maritn/Documents/dev_ws/src/fast_robot_interface_ros2/vscode/trajectory.csv");
            lin_ = read_from_file(in_file);
            in_file.close();

            counter_ = 0;
            
        }

    private:
        void timer_callback() {
            // perform ptp motion
            std::vector<double> out;
            if (counter_ < ptp_.size()) {
                out = ptp_[counter_];
            }
            else {
                out = lin_[counter_ - ptp_.size()];
            }
            counter_++;

            RCLCPP_INFO(this->get_logger(), "publishing joint angle (%f, %f, %f, %f, %f, %f, %f,)", 
                out[0], 
                out[1], 
                out[2], 
                out[3], 
                out[4], 
                out[5], 
                out[6]
            );

            auto msg = std_msgs::msg::Float64MultiArray();
            msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "i";

            msg.data.clear();
            msg.data.insert(msg.data.end(), out.begin(), out.end());

            pub_->publish(msg);
        };

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        std::vector<std::vector<double>> ptp_;
        std::vector<std::vector<double>> lin_;
        int counter_;
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PTPPublisher>());
    rclcpp::shutdown();
    return 0;
}