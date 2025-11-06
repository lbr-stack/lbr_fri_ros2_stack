#include "lbr_fri_ros2/filters.hpp"

namespace lbr_fri_ros2 {
ExponentialFilter::ExponentialFilter()
    : tau_(std::numeric_limits<double>::quiet_NaN()),
      sample_time_(std::numeric_limits<double>::quiet_NaN()),
      alpha_(std::numeric_limits<double>::quiet_NaN()) {}

ExponentialFilter::ExponentialFilter(const double &tau) : tau_(tau) {}

void ExponentialFilter::initialize(const double &sample_time) {
  if (std::isnan(tau_)) {
    throw std::runtime_error("Time constant must be set before initializing.");
  }
  return initialize(tau_, sample_time);
}

void ExponentialFilter::initialize(const double &tau, const double &sample_time) {
  if (tau <= 0.0) {
    throw std::runtime_error("Time constant must be positive and greater zero.");
  }
  if (sample_time < 0.0) {
    throw std::runtime_error("Sample time must be positive.");
  }
  double alpha = 1.0 - std::exp(-sample_time / tau);
  if (!validate_alpha_(alpha)) {
    throw std::runtime_error("Alpha is not within [0, 1]");
  }
  tau_ = tau;
  sample_time_ = sample_time;
  alpha_ = alpha;
}

bool ExponentialFilter::validate_alpha_(const double &alpha) { return alpha <= 1. && alpha >= 0.; }
} // namespace lbr_fri_ros2
