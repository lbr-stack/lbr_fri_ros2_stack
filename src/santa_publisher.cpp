#include <chrono>
#include <math.h>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fri/friLBRState.h>

using namespace std::chrono_literals;

// create point cloud for santas house
using point = std::tuple<double, double>;
using point_cloud = std::vector<point>;

point_cloud santas_house = {
	std::make_tuple<double, double>( 0.,  0.),// 1
	std::make_tuple<double, double>(10.,  0.),// 2
	std::make_tuple<double, double>(10., 20.),// 3
	std::make_tuple<double, double>( 0.,  0.),// 4
	std::make_tuple<double, double>( 0., 20.),// 5
	std::make_tuple<double, double>(10., 20.),// 6
	std::make_tuple<double, double>( 5., 25.),// 7
	std::make_tuple<double, double>( 0., 20.),// 8
	std::make_tuple<double, double>(10.,  0.),// 9
}; // in units of mm, defined in initial plane

// create ramp function with maximum velocity and defined acceleration
class RampFunction {
public:
	RampFunction(double s /*distance*/, double a /*acceleration*/, double vmax /*maximum velocity*/) s_(s), a_(a), vmax_(vmax) {
		
		// compute t1, t2, and tend
		// is vmax reached?
		double sabs = abs(s);
		if (vmax*vmax/a > sabs) { // vmax not reached
			t1_ = sqrt(sabs/a);
			t2_ = t1_;
			tend_ = 2*t1_;
		}
		else { // vmax reached
			t1_ = vmax/a;
			tend_ = sabs/vmax + vmax/a;
			t2_ = t_end_ - t1_;
		}
	};

	~RampFunction() {	};

	void update(double s /*distance*/) {
		// compute t1, t2, and tend
		// is vmax reached?
		double sabs = abs(s);
		if (vmax_*vmax_ / a_ > sabs) { // vmax not reached
			t1_ = sqrt(sabs / a_);
			t2_ = t1_;
			tend_ = 2 * t1_;
		}
		else { // vmax reached
			t1_ = vmax_ / a_;
			tend_ = sabs / vmax_ + vmax_ / a_;
			t2_ = t_end_ - t1_;
		}
	};

	double getEndTime() const {
		return t_end_;
	};

	double s(double t) {
		if (t < t1_) {
			return 0.5*a_*t*t;
		}
		else if (t >= t2_) {
			return s_ - 0.5*(t_end_ - t)*(t_end_ - t)*a_;
		}
		else {
			return 0.5*a_*t1_*t1_ + vmax_ * (t - t1_);
		}
	};
private:
	double s_, a_, vmax_;
	double t1_, t2_, tend_;
};

// create LIN motion
class LIN {
public:
	LIN(const double* x_start, const double* x_end, const double* q_init, const double dt, const int n_joints, const double a = .15, const double vmax = .1, const double dt = 0.005) : n_joints_(n_joints) {
		q_ = new double[n_joints];
		x_start_ = new double[3];
		x_end_ = new double[3];

		// init pos values
		for (int i = 0; i < 3; i++) {
			x_[i] = x_start[i];
			x_start_[i] = x_start[i];
			x_end_[i] = x_end[i];
		}

		// init joint angles
		for (int i = 0; i < n_joints; i++) {
			q_[i] = q_init[i];
		}

		double distance = sqrt((x_start[0] - x_end[0])*(x_start[0] - x_end[0]) + 
							   (x_start[1] - x_end[1])*(x_start[1] - x_end[1]) +
							   (x_start[2] - x_end[2])*(x_start[2] - x_end[2]));
		rf_ = new RampFunction(distance, a, vmax);
	};
	~LIN() {
		delete[] q_;
		delete[] q_start_;
		delete[] q_end_;

		delete rf_;
	};

	std::vector<std::vector<double>> compute() {
		double t_end = rf_->getEndTime();

		double distance = sqrt((x_start[0] - x_end[0])*(x_start[0] - x_end[0]) +
							   (x_start[1] - x_end[1])*(x_start[1] - x_end[1]) +
							   (x_start[2] - x_end[2])*(x_start[2] - x_end[2]));

		std::vector<std::vector> qs;

		for (double t = 0; t < t_end; t += dt_) {
			for (int i = 0; i < 3; i++) {
				x_[i] += (x_start_[i] - x_end_[i]) / distance * rf_->s(t);
			}

			double cartInput[7] = { x[0], x[1], x[2], 0., 0., 0., 0. };
			int status = 0;
			int turn = 0;

			// inverse kinematics
			if (!getInverse(cartInput, status, turn, q_ /*update*/, q_ /*previous*/, NULL, NULL)) {
				printf("could not invert\n");
			}

			qs_.push_back(q_);
		}

		return qs;
	};

	bool update(const double* x_start, const double* x_end, const double* q_init) {
		// init pos values
		for (int i = 0; i < 3; i++) {
			x_[i] = x_start[i];
			x_start_[i] = x_start[i];
			x_end_[i] = x_end[i];
		}

		// init joint angles
		for (int i = 0; i < n_joints_; i++) {
			q_[i] = q_init[i];
		}

		// update ramp function
		double distance = sqrt((x_start[0] - x_end[0])*(x_start[0] - x_end[0]) +
							   (x_start[1] - x_end[1])*(x_start[1] - x_end[1]) +
							   (x_start[2] - x_end[2])*(x_start[2] - x_end[2]));
		rf_->update(distance);
	}; // update target
private:
	const int n_joints_;
	double* q_;
	double* x_;
	double* x_start_;
	double* x_end_;
	const double dt_;

	RampFunction* rf_;
};



class Publisher : public rclcpp::Node {

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
			// hold position in the beginning


			// follow point cloud




            RCLCPP_INFO(this->get_logger(), "%f", _offset);

            // calculate new offset
            double newOffset = _amplRad * sin(_phi);
            _offset = _offset * _filterCoeff + newOffset * (1.0 - _filterCoeff);
            _phi += _stepWidth;
            if (_phi >= 2 * M_PI) _phi -= 2 * M_PI;     

            std::vector<double> out{_offset, 0., 0., 0., 0., 0., 0.};

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

        double _stepWidth;      //!< stepwidth for sine 
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}