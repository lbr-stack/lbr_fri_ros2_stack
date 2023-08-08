#include "lbr_fri_ros2/lbr_utils.hpp"

namespace lbr_fri_ros2 {

ExponentialFilter::ExponentialFilter() : ExponentialFilter::ExponentialFilter(0, 0.0) {}

ExponentialFilter::ExponentialFilter(const std::uint16_t &cutoff_frequency,
                                     const double &sample_time) {
  set_cutoff_frequency(cutoff_frequency, sample_time);
}

inline double ExponentialFilter::compute(const double &current, const double &previous) {
  return filters::exponentialSmoothing(current, previous, smoothing_factor_);
}

void ExponentialFilter::set_cutoff_frequency(const std::uint16_t &cutoff_frequency,
                                             const double &sample_time) {
  cutoff_frequency_ = cutoff_frequency;
  if (cutoff_frequency_ > static_cast<uint16_t>(1. / sample_time)) {
    cutoff_frequency_ = static_cast<uint16_t>(1. / sample_time);
  }
  sample_time_ = sample_time;
  smoothing_factor_ = compute_smoothing_factor(cutoff_frequency, sample_time);
  if (!validate_smoothing_factor(smoothing_factor_)) {
    throw std::runtime_error("Smoothing factor is not within [0, 1].");
  }
}

inline const double &ExponentialFilter::get_sample_time() const { return sample_time_; }

inline const double &ExponentialFilter::get_smoothing_factor() const { return smoothing_factor_; }

double ExponentialFilter::compute_smoothing_factor(const std::uint16_t &cutoff_frequency,
                                                   const double &sample_time) {
  double omega_3db = 2.0 * M_PI * sample_time * cutoff_frequency;
  return std::cos(omega_3db) - 1 +
         std::sqrt(std::pow(std::cos(omega_3db), 2) - 4 * std::cos(omega_3db) + 3);
}

bool ExponentialFilter::validate_smoothing_factor(const double &smoothing_factor) {
  return smoothing_factor <= 1. && smoothing_factor >= 0.;
}

LBRFilter::LBRFilter(const rclcpp::Node::SharedPtr node)
    : LBRFilter(node->get_node_logging_interface(), node->get_node_parameters_interface()) {}

LBRFilter::LBRFilter(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface)
    : logging_interface_(logging_interface), parameter_interface_(parameter_interface) {}

void LBRFilter::compute(const double *const current, JointArrayType &previous) {
  int i = 0;
  std::for_each(current, current + KUKA::FRI::LBRState::NUMBER_OF_JOINTS,
                [&](const auto &current_i) {
                  previous[i] = exponential_filter_.compute(current_i, previous[i]);
                  ++i;
                });
}

void LBRFilter::init(const std::uint16_t &cutoff_frequency, const double &sample_time) {
  if (!parameter_interface_->has_parameter("cutoff_frequency")) {
    parameter_interface_->declare_parameter("cutoff_frequency",
                                            rclcpp::ParameterValue(cutoff_frequency));
  }
  exponential_filter_.set_cutoff_frequency(cutoff_frequency, sample_time);

  parameter_callback_handle_ = parameter_interface_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          try {
            if (parameter.get_name() == "cutoff_frequency") {
              exponential_filter_.set_cutoff_frequency(parameter.as_int(),
                                                       exponential_filter_.get_sample_time());
              RCLCPP_INFO(logging_interface_->get_logger(),
                          "Set %s to: %ld, new smoothing factor: %f.", parameter.get_name().c_str(),
                          parameter.as_int(), exponential_filter_.get_smoothing_factor());
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

} // end of namespace lbr_fri_ros2
