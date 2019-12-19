#include <chrono>
#include <math.h>
#include <tuple>
#include <vector>
#include <cmath>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// FRI
#include <fri/friLBRState.h>

// custom
#include <ramp_function.h>

using namespace std::chrono_literals;

auto PTP(double* q_start, double* q_end, double dt, double a, double vmax) -> std::vector<std::vector<double>> {

	double dq[KUKA::FRI::NUMBER_OF_JOINTS]; // defaults zero

	for (int i = 0; i < KUKA::FRI::NUMBER_OF_JOINTS; i++) {
		dq = q_start - q_end;
	}

	// find longest for scaling ptp motion
	double longest = 0.;
	for (int i = 0; i < KUKA::FRI::NUMBER_OF_JOINTS; i++) {
		if (std::abs(dq[i]) > longest) {
			longest = std::abs(dq[i]);
		}
	}

	double a = 0.;
	double vmax = 0.;
	RampFunction rf(longest, a, vmax);

	double tend = rf.getEndTime();
	std::vector<double> q;
	std::vector<std::vector<double>> trajectory;

	for (double t = 0.; t < tend; t+=dt) {
		for (int i = 0; i < KUKA::FRI::NUMBER_OF_JOINTS; i++) {
			q[i] = q_start[i] + dq[i]/longest*rf.s(t);
		}
		trajectory.push_back(q);
	}
	return trajectory;
}

namespace KUKA {
namespace FRI {
	
class Publisher : public rclcpp::Node, public LBRClient {

    public:
        Publisher() 
            : Node("fri_test_publisher"),
              _stepWidth(0.0) {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lbr_joint_angle_in", 10);
            timer_ = this->create_wall_timer(5ms, std::bind(&Publisher::timer_callback, this));   

            // copy KUKA's example
            _stepWidth = 0.005; // robotState().getSampleTime();
        }

    private:
        void timer_callback() {

// read current position and perform ptp motion to initial position
// read in lin motion from csv file and publish to topic






			// collect output joint angles
            std::vector<double> out{_offset, 0., 0., 0., 0., 0., 0.};

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

        double _stepWidth;      //!< stepwidth for sine 
};

} // FRI
} // KUKA


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}