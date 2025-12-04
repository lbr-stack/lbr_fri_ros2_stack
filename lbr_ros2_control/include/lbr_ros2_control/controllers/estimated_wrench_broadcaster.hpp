#ifndef LBR_ROS2_CONTROL__ESTIMATED_WRENCH_BROADCASTER_HPP_
#define LBR_ROS2_CONTROL__ESTIMATED_WRENCH_BROADCASTER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_fri_ros2/wrench_estimator.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class EstimatedWrenchBroadcaster : public controller_interface::ChainableControllerInterface {
public:
  EstimatedWrenchBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool assign_state_interfaces_();
  void release_state_interfaces_();
  void configure_joint_names_();
  void configure_parameters_();
  bool read_state_interfaces_();

  // force-torque estimation
  lbr_fri_ros2::WrenchEstimatorParameters wrench_estimator_parameters_;
  std::unique_ptr<lbr_fri_ros2::WrenchEstimator> wrench_estimator_ptr_;
  lbr_fri_ros2::cart_array_t wrench_; // zero or fill with nans?

  // joint names
  lbr_fri_ros2::jnt_name_array_t joint_names_;

  // referenced by state interfaces
  lbr_fri_ros2::jnt_array_t joint_positions_, external_torques_; // fill with nans...

  // state interfaces (used by this controller to estimate wrenches)
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_, external_torque_state_interfaces_;

  // force-torque publisher (in unchained mode)
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      rt_wrench_publisher_ptr_;
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__ESTIMATED_WRENCH_BROADCASTER_HPP_
