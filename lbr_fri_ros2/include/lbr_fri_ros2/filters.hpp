#ifndef LBR_FRI_ROS2__FILTERS_HPP_
#define LBR_FRI_ROS2__FILTERS_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "control_toolbox/filters.hpp"
#include "control_toolbox/pid_ros.hpp"

#include "friLBRClient.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
class ExponentialFilter {
public:
  /**
   * @brief Construct a new Exponential Filter object. Performs exponential smoothing with a
   * #cutoff_frequency_ according to
   * https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency.
   *
   */
  ExponentialFilter();

  /**
   * @brief Construct a new Exponential Filter object. Performs exponential smoothing with a
   * #cutoff_frequency_ according to
   * https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency.
   *
   * @param[in] cutoff_frequency Frequency in Hz.
   * @param[in] sample_time Sample time in seconds.
   */
  ExponentialFilter(const double &cutoff_frequency, const double &sample_time);

  /**
   * @brief Compute the exponential smoothing using the control_toolbox
   * https://github.com/ros-controls/control_toolbox.
   *
   * @param[in] current The current value.
   * @param[in] previous The previous smoothed value.
   * @return double The returned smoothed value.
   */
  inline double compute(const double &current, const double &previous) {
    return filters::exponentialSmoothing(current, previous, alpha_);
  };

  /**
   * @brief Set the cutoff frequency object. Internally computes the new #alpha_.
   *
   * @param[in] cutoff_frequency Frequency in Hz.
   * @param[in] sample_time Sample time in seconds.
   */
  void set_cutoff_frequency(const double &cutoff_frequency, const double &sample_time);

  /**
   * @brief Get #sample_time_.
   *
   * @return const double&
   */
  inline const double &get_sample_time() const { return sample_time_; };

  /**
   * @brief Get #alpha_.
   *
   * @return const double&
   */
  inline const double &get_alpha() const { return alpha_; };

protected:
  /**
   * @brief Compute alpha given the cutoff frequency and the sample time.
   *
   * @param[in] cutoff_frequency Frequency in Hz.
   * @param[in] sample_time Sample time in seconds.
   * @return double Alpha based on
   * https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency.
   */
  double compute_alpha_(const double &cutoff_frequency, const double &sample_time);

  /**
   * @brief Validate alpha in [0, 1].
   *
   * @param[in] alpha Alpha parameter for smoothing.
   * @return true if in [0, 1].
   * @return false if outside [0, 1].
   */
  bool validate_alpha_(const double &alpha);

  double cutoff_frequency_; /**< Frequency in Hz.*/
  double sample_time_;      /**< Sample time in seconds.*/
  double
      alpha_; /**< Alpha parameter based on
                 https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency.*/
};

class JointExponentialFilterArray {
  using value_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointExponentialFilterArray() = default;

  void compute(const double *const current, value_array_t &previous);
  void initialize(const double &cutoff_frequency, const double &sample_time);
  inline const bool &is_initialized() const { return initialized_; };

protected:
  bool initialized_{false};              /**< True if initialized.*/
  ExponentialFilter exponential_filter_; /**< Exponential filter applied to all joints.*/
};

struct PIDParameters {
  double p{0.0};          /**< Proportional gain.*/
  double i{0.0};          /**< Integral gain.*/
  double d{0.0};          /**< Derivative gain.*/
  double i_max{0.0};      /**< Maximum integral value.*/
  double i_min{0.0};      /**< Minimum integral value.*/
  bool antiwindup{false}; /**< Antiwindup enabled.*/
};

class JointPIDArray {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::JointPIDArray";
  using value_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using pid_array_t = std::array<control_toolbox::Pid, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointPIDArray() = default;

  void compute(const value_array_t &command_target, const value_array_t &state,
               const std::chrono::nanoseconds &dt, value_array_t &command);
  void compute(const value_array_t &command_target, const double *state,
               const std::chrono::nanoseconds &dt, value_array_t &command);
  void initialize(const PIDParameters &pid_parameters, const double &dt);
  inline const bool &is_initialized() const { return initialized_; };

  void log_info() const;

protected:
  bool initialized_{false};     /**< True if initialized.*/
  pid_array_t pid_controllers_; /**< PID controllers for each joint.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__FILTERS_HPP_
