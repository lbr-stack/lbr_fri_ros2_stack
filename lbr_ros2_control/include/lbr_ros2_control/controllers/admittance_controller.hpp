#ifndef LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_

#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friLBRState.h"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class Admittance {

}

class AdmittanceController : public controller_interface::ControllerInterface {
  static constexpr uint8_t CARTESIAN_DOF = 6;

public:
  AdmittanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool reference_command_interfaces_();
  bool reference_state_interfaces_();
  void clear_command_interfaces_();
  void clear_state_interfaces_();

  std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS> joint_names_ = {
      "A1", "A2", "A3", "A4", "A5", "A6", "A7"};

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      joint_position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      estimated_ft_sensor_state_interface_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
