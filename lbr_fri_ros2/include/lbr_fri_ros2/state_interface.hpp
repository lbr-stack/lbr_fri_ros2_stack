#ifndef LBR_FRI_ROS2__STATE_INTERFACE_HPP_
#define LBR_FRI_ROS2__STATE_INTERFACE_HPP_
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
class StateInteface {
protected:
  // ROS types
  using ros_state_type = lbr_fri_msgs::msg::LBRState;
  using ros_joint_pos_type = ros_state_type::_measured_joint_position_type;
  using const_ros_joint_pos_type_ref = const ros_joint_pos_type &;

  // FRI types
  using fri_state_type = KUKA::FRI::LBRState;
  using const_fri_state_type_ref = const fri_state_type &;
  using fri_session_state_type = KUKA::FRI::ESessionState;

public:
  StateInteface() = default;

  inline const ros_state_type &get_state() const { return state_; };

  void set_state(const_fri_state_type_ref state);
  void set_state_open_loop(const_fri_state_type_ref state,
                           const_ros_joint_pos_type_ref joint_position);

protected:
  ros_state_type state_;
  // JointExponentialFilterArrayROS external_torque_filter_;
  // JointExponentialFilterArrayROS measured_torque_filter_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__STATE_INTERFACE_HPP_
