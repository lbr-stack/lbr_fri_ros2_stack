#include <chrono>
#include <math.h>

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

class PTPPublisher : public rclcpp::Node {

    public:
        PTPPublisher() 
            : Node("fri_test_publisher") {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lbr_joint_angles_in", 10);
            timer_ = this->create_wall_timer(5ms, std::bind(&PTPPublisher::timer_callback, this));
            double q_start[7] = {0., 0., 0., 0., 0., 0., 0.};
            double q_end[7] = {0., 60.*DEG2RAD, 0., -60*DEG2RAD, 0., 60.*DEG2RAD, 0.};
            traj_ = PTP(q_start, q_end, 0.005); 
            counter_ = 0;
            
        }

    private:
        void timer_callback() {
            std::vector<double> out = traj_[counter_];
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
        std::vector<std::vector<double>> traj_;
        int counter_;
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PTPPublisher>());
    rclcpp::shutdown();
    return 0;
}