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
  ExponentialFilter();
  ExponentialFilter(const double &cutoff_frequency, const double &sample_time);

  inline double compute(const double &current, const double &previous);
  void set_cutoff_frequency(const double &cutoff_frequency, const double &sample_time);
  inline const double &get_sample_time() const;
  inline const double &get_alpha() const;

protected:
  // https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency
  double compute_alpha(const double &cutoff_frequency, const double &sample_time);
  bool validate_alpha(const double &alpha);

  double cutoff_frequency_;
  double sample_time_;
  double alpha_;
};

class JointExponentialFilterArrayROS {
  using ValueArrayType = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointExponentialFilterArrayROS() = delete;
  JointExponentialFilterArrayROS(const rclcpp::Node::SharedPtr node,
                                 const std::string &param_prefix = "");
  JointExponentialFilterArrayROS(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
      const std::string &param_prefix = "");

  void compute(const double *const current, ValueArrayType &previous);
  void init(const double &cutoff_frequency, const double &sample_time);

protected:
  ExponentialFilter exponential_filter_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface_;
  std::string param_prefix_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

class JointPIDArrayROS {
  using ValueArrayType = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using NameArrayType = std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using PIDArrayType = std::array<control_toolbox::PidROS, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;

public:
  JointPIDArrayROS() = delete;
  JointPIDArrayROS(const rclcpp::Node::SharedPtr node, const NameArrayType &names,
                   const std::string &prefix = "");

  void compute(const ValueArrayType &command_target, const ValueArrayType &state,
               const rclcpp::Duration &dt, ValueArrayType &command);

  void init(const double &p, const double &i, const double &d, const double &i_max,
            const double &i_min, const bool &antiwindup);

protected:
  PIDArrayType pid_controllers_;
};

} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__UTILS_HPP_
