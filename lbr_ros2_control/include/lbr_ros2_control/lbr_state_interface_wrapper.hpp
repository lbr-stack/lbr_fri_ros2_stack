#ifndef LBR_ROS2_CONTROL__LBR_STATE_INTERFACE_REFERENCE_HPP_
#define LBR_ROS2_CONTROL__LBR_STATE_INTERFACE_REFERENCE_HPP_

#include <functional>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_ros2_control/lbr_system_interface_type_values.hpp"

namespace lbr_ros2_control {
struct LBRStateInterfaceWrapper {
  using loaned_state_interface_t = std::reference_wrapper<hardware_interface::LoanedStateInterface>;

  // override lhs = to
  void operator=(lbr_fri_msgs::msg::LBRState &lhs) {
    // lhs.client_command_mode = static_cast<int8_t>(client_command_mode.get().get_value());
    // lhs.client_command_mode = client_command_mode.
  }

  // map: names -> values
  // for state interfaces:
  //   map[state interface name] = state interface value
  // 
  // msg.name = map[name]

  // loaned_state_interface_t client_command_mode;
  std::vector<loaned_state_interface_t> commanded_joint_position;
  std::vector<loaned_state_interface_t> commanded_torque;
  // loaned_state_interface_t connection_quality;
  // loaned_state_interface_t control_mode;
  // loaned_state_interface_t drive_state;
  // loaned_state_interface_t external_torque;
  // loaned_state_interface_t ipo_joint_position;
  // loaned_state_interface_t measured_joint_position;
  // loaned_state_interface_t measured_torque;
  // loaned_state_interface_t overlay_type;
  // loaned_state_interface_t safety_state;
  // loaned_state_interface_t sample_time;
  // loaned_state_interface_t session_state;
  // loaned_state_interface_t time_stamp_nano_sec;
  // loaned_state_interface_t time_stamp_sec;
  // loaned_state_interface_t tracking_performance;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_STATE_INTERFACE_REFERENCE_HPP_