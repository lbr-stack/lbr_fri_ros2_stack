#ifndef LBR_FRI_ROS2_FACTORIES_HPP
#define LBR_FRI_ROS2_FACTORIES_HPP

#include <memory>

#include "friLBRClient.h"

#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
std::unique_ptr<KUKA::FRI::LBRClient> client_factory(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
    const std::string &variant = "position");
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2_FACTORIES_HPP
