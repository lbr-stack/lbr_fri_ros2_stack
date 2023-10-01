#ifndef LBR_FRI_ROS2__STATE_INTERFACE_HPP_
#define LBR_FRI_ROS2__STATE_INTERFACE_HPP_
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/utils.hpp"

namespace lbr_fri_ros2 {
class StateInterface {
protected:
  // ROS IDL types
  using idl_state_t = lbr_fri_msgs::msg::LBRState;
  using const_idl_state_t_ref = const idl_state_t &;
  using idl_joint_pos_t = idl_state_t::_measured_joint_position_type;
  using const_idl_joint_pos_t_ref = const idl_joint_pos_t &;

  // FRI types
  using fri_state_t = KUKA::FRI::LBRState;
  using const_fri_state_t_ref = const fri_state_t &;
  using fri_session_state_t = KUKA::FRI::ESessionState;

public:
  StateInterface() = delete;
  StateInterface(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr);

  inline const_idl_state_t_ref &get_state() const { return state_; };

  void set_state(const_fri_state_t_ref state);
  void set_state_open_loop(const_fri_state_t_ref state, const_idl_joint_pos_t_ref joint_position);

protected:
  void init_filters_();

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  idl_state_t state_;
  JointExponentialFilterArrayROS external_torque_filter_;
  JointExponentialFilterArrayROS measured_torque_filter_;
  bool filters_init_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__STATE_INTERFACE_HPP_
