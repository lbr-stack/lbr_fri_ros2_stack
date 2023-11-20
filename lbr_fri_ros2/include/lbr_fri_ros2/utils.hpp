#ifndef LBR_FRI_ROS2__UTILS_HPP_
#define LBR_FRI_ROS2__UTILS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "control_toolbox/filters.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

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

class JointExponentialFilterArrayROS {
  using value_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointExponentialFilterArrayROS() = delete;

  /**
   * @brief Construct a new JointExponentialFilterArrayROS object.
   *
   * @param[in] logging_interface Logging interface.
   * @param[in] parameter_interface Parameter interface.
   * @param[in] param_prefix Parameter prefix is e.g. used as: param_prefix + "." +
   * "cut_off_frequency".
   */
  JointExponentialFilterArrayROS(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
      const std::string &param_prefix = "");

  /**
   * @brief Compute the exponential smoothing for each joints using #exponential_filter_.
   *
   * @param[in] current The current joint values.
   * @param[in, out] previous The previous smoothed joint values. Will be updated.
   */
  void compute(const double *const current, value_array_t &previous);
  void init(const double &cutoff_frequency, const double &sample_time);
  inline const std::string &param_prefix() const { return param_prefix_; }

protected:
  const std::string cutoff_frequency_param_name_ = "cutoff_frequency"; /**< Parameter name.*/
  ExponentialFilter exponential_filter_; /**< Exponential filter applied to all joints.*/
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
      logging_interface_; /**< Logging interface.*/
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
      parameter_interface_;  /** Parameter interface.*/
  std::string param_prefix_; /** Parameter prefix is used as "param_prefix" + "." + "param_name".*/
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_; /**< Parameter callback handle to update parameters.*/
};

class JointPIDArrayROS {
  using value_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using name_array_t = std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using pid_array_t = std::array<control_toolbox::PidROS, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointPIDArrayROS() = delete;

  /**
   * @brief Construct a new JointPIDArrayROS object. Used to control the LBRs joints. The parameters
   * / topics are prefixed as prefix + names[i].
   *
   * @param[in] node Shared node.
   * @param[in] names Names of the joints.
   * @param[in] prefix Prefix for the parameters.
   */
  JointPIDArrayROS(const rclcpp::Node::SharedPtr node, const name_array_t &names,
                   const std::string &prefix = "");

  /**
   * @brief Compute the PID update.
   *
   * @param[in] command_target The target joint command.
   * @param[in] state The current joint state.
   * @param[in] dt The time step.
   * @param[out] command The returned joint command.
   */
  void compute(const value_array_t &command_target, const value_array_t &state,
               const rclcpp::Duration &dt, value_array_t &command);

  /**
   * @brief Compute the PID update.
   *
   * @param[in] command_target The target joint command.
   * @param[in] state The current joint state.
   * @param[in] dt The time step.
   * @param[out] command The returned joint command.
   */
  void compute(const value_array_t &command_target, const double *state, const rclcpp::Duration &dt,
               value_array_t &command);

  /**
   * @brief Initialize the PID controllers. Sets all #pid_controllers_ to the same parameters, but
   * can be overriden individually.
   *
   * @param[in] p The proportional gain.
   * @param[in] i The integral gain.
   * @param[in] d The derivative gain.
   * @param[in] i_max The maximum integral value.
   * @param[in] i_min The minimum integral value.
   * @param[in] antiwindup Antiwindup enabled or disabled.
   */
  void init(const double &p, const double &i, const double &d, const double &i_max,
            const double &i_min, const bool &antiwindup);

protected:
  pid_array_t pid_controllers_; /**< PID controllers for each joint.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__UTILS_HPP_
