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

auto PTP(double* q_start, const double* q_end, bool hold, double hold_time = 10., double dt = 0.005, double a = 0.1, double vmax = 0.2) -> std::vector<std::vector<double>> {

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

    if (hold) {
        for (double t = 0.; t < hold_time; t+=dt) { // do nothin for hold_time seconds
            for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
			    q[i] = q_start[i];
		    }
            trajectory.push_back(q);
        }
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

            // read in linear motion
            std::fstream in_file;
            // in_file.open("/home/maritn/Documents/dev_ws/src/fast_robot_interface_ros2/vscode/trajectory_rvim.csv");
            in_file.open("/home/maritn/Documents/dev_ws/src/fast_robot_interface_ros2/vscode/trajectory_merry_christmas.csv");
            lin_ = read_from_file(in_file);
            in_file.close();

            // create ptp motion
            double q_start[7] = {0., 0., 0., 0., 0., 0., 0.};
            double q_end[7] = {0., 0., 0., 0., 0., 0., 0.};
            for (int i = 0; i < lin_[0].size(); i++) {
                q_end[i] = lin_[0][i];
            }
            ptp_ = PTP(q_start, q_end, true /*hold still*/, 10. /*for 10 seconds*/, 0.005); 

            // watch camera ptp motion
            int last = lin_.size() - 1;
            for (int i = 0; i < lin_[0].size(); i++) {
                q_start[i] = lin_[last][i];
            }

            q_end[0] =  18.0*DEG2RAD;
            q_end[1] =  53.5*DEG2RAD;
            q_end[2] = - 3.8*DEG2RAD;
            q_end[3] = -72.5*DEG2RAD;
            q_end[4] =  55.7*DEG2RAD;
            q_end[5] = -77.3*DEG2RAD;
            q_end[6] = -24.4*DEG2RAD;

            ptp_watch_camera_ = PTP(q_start, q_end, 0.005, 0.4, 0.8); 

            // watch letter ptp motion
            for (int i = 0; i < lin_[0].size(); i++) {
                q_start[i] = q_end[i];
            }

            q_end[0] =   18.0*DEG2RAD;
            q_end[1] =   42.2*DEG2RAD;
            q_end[2] =   20.9*DEG2RAD;
            q_end[3] = - 72.9*DEG2RAD;
            q_end[4] =  127*DEG2RAD;
            q_end[5] = - 66.3*DEG2RAD;
            q_end[6] = -24.4*DEG2RAD;

            ptp_watch_letter_ = PTP(q_start, q_end, true /*hold still*/, 2. /*for 10 seconds*/, 0.005, 0.4, 0.8); 

            // init counter
            counter_ = 0;
            
        }

    private:
        void timer_callback() {
            // perform ptp motion to start
            std::vector<double> out;
            if (counter_ < ptp_.size()) {
                out = ptp_[counter_];
            }
            else if (counter_ - ptp_.size() < lin_.size()) {
                out = lin_[counter_ - ptp_.size()];
            }
            else if (counter_ - ptp_.size() - lin_.size() < ptp_watch_camera_.size()) { // watch camera
                out = ptp_watch_camera_[counter_ - ptp_.size() - lin_.size()];
            }
            else if (counter_ - ptp_.size() - lin_.size() - ptp_watch_camera_.size() < ptp_watch_letter_.size()) { // watch letter
                out = ptp_watch_letter_[counter_ - ptp_.size() - lin_.size() - ptp_watch_camera_.size()];
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
        std::vector<std::vector<double>> lin_;
        std::vector<std::vector<double>> ptp_;
        std::vector<std::vector<double>> ptp_watch_camera_;
        std::vector<std::vector<double>> ptp_watch_letter_;
        int counter_;
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PTPPublisher>());
    rclcpp::shutdown();
    return 0;
}