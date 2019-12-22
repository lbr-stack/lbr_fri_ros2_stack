#pragma once

#include <vector>
#include <cmath>

#include <KinematicsLib.h>

#include <RampFunction.h>

// create LIN motion
class LIN {
public:
	LIN(const double* x_start, const double* x_end, const double* q_init, const int n_joints, const int status, const int turn, const double a = 120./*mm/s^2*/, const double vmax = 240. /*mm/s*/, const double dt = 0.005) : n_joints_(n_joints), status_(status), turn_(turn), dt_(dt) {
		q_ = new double[n_joints];
		x_ = new double[7];
		x_start_ = new double[7];
		x_end_ = new double[7];

		// init pos values
		for (int i = 0; i < 7; i++) {
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
		delete[] x_;
		delete[] x_start_;
		delete[] x_end_;

		delete rf_;
	};

	std::vector<std::vector<double>> compute() {
		double t_end = rf_->getEndTime();

		double distance = sqrt((x_start_[0] - x_end_[0])*(x_start_[0] - x_end_[0]) +
			(x_start_[1] - x_end_[1])*(x_start_[1] - x_end_[1]) +
			(x_start_[2] - x_end_[2])*(x_start_[2] - x_end_[2]));

		std::vector<std::vector<double>> qs;

		bool success = true;

		for (double t = 0; t < t_end; t += dt_) {
			//printf("s(t) %f at time %f at pos %f of ditance %f\n", rf_->s(t), t, x_[0], distance);
			for (int i = 0; i < 3; i++) {
				x_[i] = x_start_[i] + (x_end_[i] - x_start_[i]) / distance * rf_->s(t);
			}

			// inverse kinematics
			double q_init[7] = { q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6], };
			if (!getInverse(x_, status_, turn_, q_ /*update*/, q_init /*previous*/, NULL, NULL)) {
				printf("could not invert\n");
				system("pause");
				success = false;
				break;
			}

			std::vector<double> q{ q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6] };
			qs.push_back(q);
		}

		if (success) {
			printf("computed trajectory of length %d at frequency %d Hz to total time %f s\n", qs.size(), int(1/dt_), rf_->getEndTime());
			//system("pause");
		}
		else {
			printf("failed computing trajectory\n");
			system("pause");
		}

		return qs;
	};

	void update(const double* x_start, const double* x_end, const double* q_init) {
		// init pos values
		for (int i = 0; i < 7; i++) {
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
	double* q_;
	double* x_;
	double* x_start_;
	double* x_end_;
	const int n_joints_;
	const int status_;
	const int turn_;
	const double dt_;

	RampFunction* rf_;
};