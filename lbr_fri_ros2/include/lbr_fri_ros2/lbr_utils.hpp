#ifndef LBR_FRI_ROS2__LBR_UTILS_HPP_
#define LBR_FRI_ROS2__LBR_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "control_toolbox/filters.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
class ExponentialFilter {
public:
  /**
   * @brief Construct a new Exponential Smooth object
   *
   */
  ExponentialFilter();

  /**
   * @brief Construct a new Exponential Smooth object
   *
   * @param[in] cutoff_frequency
   * @param[in] sample_time
   */
  ExponentialFilter(const std::uint16_t &cutoff_frequency, const double &sample_time);

  /**
   * @brief
   *
   * @param[in] current
   * @param[in] previous
   * @return double
   */
  inline double compute(const double &current, const double &previous);

  /**
   * @brief Set the cutoff frequency object
   *
   * @param[in] cutoff_frequency
   * @param[in] sample_time
   */
  void set_cutoff_frequency(const std::uint16_t &cutoff_frequency, const double &sample_time);

  inline const double &get_sample_time() const;
  inline const double &get_smoothing_factor() const;

protected:
  /**
   * @brief
   * https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency
   *
   * @param[in] cutoff_frequency
   * @param[in] sample_time
   * @return double
   */
  double compute_smoothing_factor(const std::uint16_t &cutoff_frequency, const double &sample_time);

  /**
   * @brief
   *
   * @param[in] smoothing_factor
   * @return true
   * @return false
   */
  bool validate_smoothing_factor(const double &smoothing_factor);

  std::uint16_t cutoff_frequency_;
  double sample_time_;
  double smoothing_factor_;
};

class LBRFilter {
  using JointArrayType = lbr_fri_msgs::msg::LBRState::_measured_joint_position_type;

public:
  LBRFilter() = delete;
  LBRFilter(const rclcpp::Node::SharedPtr node, const std::string &param_prefix = "");
  LBRFilter(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
            const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
            const std::string &param_prefix = "");

  /**
   * @brief
   *
   * @param[in] current
   * @param[in, out] previous
   */
  void compute(const double *const current, JointArrayType &previous);

  /**
   * @brief
   *
   * @param[in] cutoff_frequency
   * @param[in] sample_time
   */
  void init(const std::uint16_t &cutoff_frequency, const double &sample_time);

protected:
  ExponentialFilter exponential_filter_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface_;
  std::string param_prefix_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_UTILS_HPP_
