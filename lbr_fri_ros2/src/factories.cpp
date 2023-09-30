#include "lbr_fri_ros2/factories.hpp"

namespace lbr_fri_ros2 {

std::unique_ptr<KUKA::FRI::LBRClient> client_factory(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
    const std::string &variant) {
  if (variant == "position") {
    return std::make_unique<Client<PositionCommandInterface>>(logger_interface,
                                                              parameters_interface);
  } else if (variant == "torque") {
    return std::make_unique<Client<TorqueCommandInterface>>(logger_interface, parameters_interface);
  } else if (variant == "wrench") {
    return std::make_unique<Client<WrenchCommandInterface>>(logger_interface, parameters_interface);
  }

  std::string err = "Invalid client variant requested.";
  RCLCPP_ERROR(logger_interface->get_logger(), err.c_str());
  throw std::invalid_argument(err);
}
} // end of namespace lbr_fri_ros2
