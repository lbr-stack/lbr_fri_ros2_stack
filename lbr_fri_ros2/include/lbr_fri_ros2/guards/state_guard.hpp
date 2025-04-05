#ifndef LBR_FRI_ROS2__STATE_GUARD_HPP_
#define LBR_FRI_ROS2__STATE_GUARD_HPP_

#include <cmath>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct StateGuardParameters {
  jnt_name_array_t joint_names;                                /**< Joint names.*/
  jnt_array_t max_external_torque{2., 2., 2., 2., 2., 2., 2.}; /**< Maximum external torque [Nm].*/
  bool external_torque_safety_check{true};                     /**< External torque safety check. */
};

class StateGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::StateGuard";

public:
  StateGuard() = default;
  StateGuard(const StateGuardParameters &state_guard_parameters);
  virtual bool is_valid_state(const_idl_state_t_ref lbr_state);

  void log_info() const;

protected:
  virtual bool state_in_external_torque_limits_(const_idl_state_t_ref lbr_state) const;

  StateGuardParameters parameters_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__STATE_GUARD_HPP_
