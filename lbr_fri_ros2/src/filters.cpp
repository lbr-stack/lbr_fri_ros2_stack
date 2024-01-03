#include "lbr_fri_ros2/filters.hpp"

namespace lbr_fri_ros2 {
ExponentialFilter::ExponentialFilter() : ExponentialFilter::ExponentialFilter(0, 0.0) {}

ExponentialFilter::ExponentialFilter(const double &cutoff_frequency, const double &sample_time) {
  set_cutoff_frequency(cutoff_frequency, sample_time);
}

void ExponentialFilter::set_cutoff_frequency(const double &cutoff_frequency,
                                             const double &sample_time) {
  cutoff_frequency_ = cutoff_frequency;
  if (cutoff_frequency_ > (1. / sample_time)) {
    cutoff_frequency_ = (1. / sample_time);
  }
  sample_time_ = sample_time;
  alpha_ = compute_alpha_(cutoff_frequency, sample_time);
  if (!validate_alpha_(alpha_)) {
    throw std::runtime_error("Alpha is not within [0, 1]");
  }
}

double ExponentialFilter::compute_alpha_(const double &cutoff_frequency,
                                         const double &sample_time) {
  double omega_3db = 2.0 * M_PI * sample_time * cutoff_frequency;
  return std::cos(omega_3db) - 1 +
         std::sqrt(std::pow(std::cos(omega_3db), 2) - 4 * std::cos(omega_3db) + 3);
}

bool ExponentialFilter::validate_alpha_(const double &alpha) { return alpha <= 1. && alpha >= 0.; }

void JointExponentialFilterArray::compute(const double *const current, value_array_t &previous) {
  std::for_each(current, current + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                [&, i = 0](const auto &current_i) mutable {
                  previous[i] = exponential_filter_.compute(current_i, previous[i]);
                  ++i;
                });
}

void JointExponentialFilterArray::initialize(const double &cutoff_frequency,
                                             const double &sample_time) {
  exponential_filter_.set_cutoff_frequency(cutoff_frequency, sample_time);
  initialized_ = true;
}

void JointPIDArray::compute(const value_array_t &command_target, const value_array_t &state,
                            const std::chrono::nanoseconds &dt, value_array_t &command) {
  std::for_each(command.begin(), command.end(), [&, i = 0](double &command_i) mutable {
    command_i += pid_controllers_[i].computeCommand(command_target[i] - state[i], dt.count());
    ++i;
  });
}

void JointPIDArray::compute(const value_array_t &command_target, const double *state,
                            const std::chrono::nanoseconds &dt, value_array_t &command) {
  std::for_each(command.begin(), command.end(), [&, i = 0](double &command_i) mutable {
    command_i += pid_controllers_[i].computeCommand(command_target[i] - state[i], dt.count());
    ++i;
  });
}

void JointPIDArray::initialize(const PIDParameters &pid_parameters, const double &dt) {
  std::for_each(pid_controllers_.begin(), pid_controllers_.end(), [&](auto &pid) {
    pid.initPid(pid_parameters.p * dt, pid_parameters.i * dt, pid_parameters.d * dt,
                pid_parameters.i_max * dt, pid_parameters.i_min * dt, pid_parameters.antiwindup);
  });
  initialized_ = true;
}
} // end of namespace lbr_fri_ros2
