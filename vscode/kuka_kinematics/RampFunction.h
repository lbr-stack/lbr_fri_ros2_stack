#pragma once

#include <cmath>

// create ramp function with maximum velocity and defined acceleration
class RampFunction {
public:
	RampFunction(double s /*distance*/, double a /*acceleration*/, double vmax /*maximum velocity*/) : s_(s), a_(a), vmax_(vmax) {
		// compute t1, t2, and tend
		// is vmax reached?
		double sabs = std::abs(s);
		if (vmax*vmax / a > sabs) { // vmax not reached
			t1_ = std::sqrt(sabs / a);
			t2_ = t1_;
			tend_ = 2 * t1_;
		}
		else { // vmax reached
			t1_ = vmax / a;
			tend_ = sabs / vmax + vmax / a;
			t2_ = tend_ - t1_;
		}
	};

	~RampFunction() {	};

	void update(double s /*distance*/) {
		// compute t1, t2, and tend
		// is vmax reached?
		double sabs = std::abs(s);
		if (vmax_*vmax_ / a_ > sabs) { // vmax not reached
			t1_ = std::sqrt(sabs / a_);
			t2_ = t1_;
			tend_ = 2 * t1_;
		}
		else { // vmax reached
			t1_ = vmax_ / a_;
			tend_ = sabs / vmax_ + vmax_ / a_;
			t2_ = tend_ - t1_;
		}
	};

	double getEndTime() const {
		return tend_;
	};

	double s(double t) {
		if (t < t1_) {
			return 0.5*a_*t*t;
		}
		else if (t >= t2_) {
			return s_ - 0.5*(tend_ - t)*(tend_ - t)*a_;
		}
		else {
			return 0.5*a_*t1_*t1_ + vmax_ * (t - t1_);
		}
	};
private:
	double s_, a_, vmax_;
	double t1_, t2_, tend_;
};
