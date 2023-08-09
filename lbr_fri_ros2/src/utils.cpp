#include "lbr_fri_ros2/utils.hpp"

namespace lbr_fri_ros2 {

ExponentialFilter::ExponentialFilter() : ExponentialFilter::ExponentialFilter(0, 0.0) {}

ExponentialFilter::ExponentialFilter(const double &cutoff_frequency, const double &sample_time) {
  set_cutoff_frequency(cutoff_frequency, sample_time);
}

inline double ExponentialFilter::compute(const double &current, const double &previous) {
  return filters::exponentialSmoothing(current, previous, alpha_);
}

void ExponentialFilter::set_cutoff_frequency(const double &cutoff_frequency,
                                             const double &sample_time) {
  cutoff_frequency_ = cutoff_frequency;
  if (cutoff_frequency_ > static_cast<uint16_t>(1. / sample_time)) {
    cutoff_frequency_ = static_cast<uint16_t>(1. / sample_time);
  }
  sample_time_ = sample_time;
  alpha_ = compute_alpha(cutoff_frequency, sample_time);
  if (!validate_alpha(alpha_)) {
    throw std::runtime_error("Alpha is not within [0, 1].");
  }
}

inline const double &ExponentialFilter::get_sample_time() const { return sample_time_; }

inline const double &ExponentialFilter::get_alpha() const { return alpha_; }

double ExponentialFilter::compute_alpha(const double &cutoff_frequency, const double &sample_time) {
  double omega_3db = 2.0 * M_PI * sample_time * cutoff_frequency;
  return std::cos(omega_3db) - 1 +
         std::sqrt(std::pow(std::cos(omega_3db), 2) - 4 * std::cos(omega_3db) + 3);
}

bool ExponentialFilter::validate_alpha(const double &alpha) { return alpha <= 1. && alpha >= 0.; }

JointExponentialFilterArrayROS::JointExponentialFilterArrayROS(const rclcpp::Node::SharedPtr node,
                                                               const std::string &param_prefix)
    : JointExponentialFilterArrayROS(node->get_node_logging_interface(),
                                     node->get_node_parameters_interface(), param_prefix) {}

JointExponentialFilterArrayROS::JointExponentialFilterArrayROS(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
    const std::string &param_prefix)
    : logging_interface_(logging_interface), parameter_interface_(parameter_interface),
      param_prefix_(param_prefix) {
  if (!param_prefix_.empty()) {
    param_prefix_ += ".";
  }
}

void JointExponentialFilterArrayROS::compute(const double *const current,
                                             ValueArrayType &previous) {
  int i = 0;
  std::for_each(current, current + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                [&](const auto &current_i) {
                  previous[i] = exponential_filter_.compute(current_i, previous[i]);
                  ++i;
                });
}

void JointExponentialFilterArrayROS::init(const double &cutoff_frequency,
                                          const double &sample_time) {
  if (!parameter_interface_->has_parameter(param_prefix_ + "cutoff_frequency")) {
    parameter_interface_->declare_parameter(param_prefix_ + "cutoff_frequency",
                                            rclcpp::ParameterValue(cutoff_frequency));
  }
  exponential_filter_.set_cutoff_frequency(cutoff_frequency, sample_time);

  parameter_callback_handle_ = parameter_interface_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          try {
            if (parameter.get_name() == param_prefix_ + "cutoff_frequency") {
              exponential_filter_.set_cutoff_frequency(parameter.as_double(),
                                                       exponential_filter_.get_sample_time());
              RCLCPP_INFO(logging_interface_->get_logger(),
                          "Set %s to: %f, new smoothing factor: %f. 0: no smoothing, 1: maximal "
                          "smoothing.",
                          parameter.get_name().c_str(), parameter.as_double(),
                          1. - exponential_filter_.get_alpha());
            }
          } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
            std::string info_msg = "Invalid parameter type: " + std::string(e.what());
            RCLCPP_INFO(logging_interface_->get_logger(), info_msg.c_str());
            result.reason = info_msg;
            result.successful = false;
          }
        }
        return result;
      });
}

JointPIDArrayROS::JointPIDArrayROS(const rclcpp::Node::SharedPtr node, const NameArrayType &names,
                                   const std::string &prefix)
    : pid_controllers_(PIDArrayType{
          control_toolbox::PidROS{node, prefix + names[0]},
          control_toolbox::PidROS{node, prefix + names[1]},
          control_toolbox::PidROS{node, prefix + names[2]},
          control_toolbox::PidROS{node, prefix + names[3]},
          control_toolbox::PidROS{node, prefix + names[4]},
          control_toolbox::PidROS{node, prefix + names[5]},
          control_toolbox::PidROS{node, prefix + names[6]},
      }) {}

void JointPIDArrayROS::compute(const ValueArrayType &command_target, const ValueArrayType &state,
                               const rclcpp::Duration &dt, ValueArrayType &command) {
  int i = 0;
  std::for_each(command.begin(), command.end(), [&](double &command_i) {
    command_i += pid_controllers_[i].computeCommand(command_target[i] - state[i], dt);
    ++i;
  });
}

void JointPIDArrayROS::init(const double &p, const double &i, const double &d, const double &i_max,
                            const double &i_min, const bool &antiwindup) {
  std::for_each(pid_controllers_.begin(), pid_controllers_.end(),
                [&](auto &pid) { pid.initPid(p, i, d, i_max, i_min, antiwindup); });
}

} // end of namespace lbr_fri_ros2
