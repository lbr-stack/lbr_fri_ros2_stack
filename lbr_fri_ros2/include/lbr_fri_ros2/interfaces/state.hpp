#ifndef LBR_FRI_ROS2__INTERFACES__STATE_HPP_
#define LBR_FRI_ROS2__INTERFACES__STATE_HPP_

#include <atomic>
#include <cstring>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/triple_buffer.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct StateInterfaceParameters {
  double external_torque_tau = 0.01; /*seconds*/
  double measured_torque_tau = 0.01; /*seconds*/
};

class StateInterface {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::StateInterface";

public:
  StateInterface() = delete;
  StateInterface(const StateInterfaceParameters &state_interface_parameters = {0.04, 0.04});

  /**
   * @brief The latest state published by the FRI thread. Real-time safe, never blocks. Called from
   * one consumer thread at a time (e.g. the controller_manager), never from the FRI thread.
   *
   * @return const_idl_state_t_ref The latest state, valid until the next call.
   */
  inline const_idl_state_t_ref get_state() {
    state_buffer_.update();
    return state_buffer_.read();
  }

  /**
   * @brief The state as last received from the robot, without copy or lock. FRI thread only.
   */
  inline const_idl_state_t_ref get_state_in_fri_thread() const { return state_; }

  /**
   * @brief Update the state from the robot and publish it to other threads. FRI thread only.
   */
  void set_state(const_fri_state_t_ref state);
  void set_state_open_loop(const_fri_state_t_ref state, const_jnt_array_t_ref joint_position);

  inline void uninitialize() { state_initialized_ = false; }
  inline bool is_initialized() const { return state_initialized_; };

  void log_info() const;

protected:
  void init_filters_();
  void publish_state_();

  std::atomic_bool state_initialized_;
  idl_state_t state_;                     /**< FRI thread only.*/
  TripleBuffer<idl_state_t> state_buffer_; /**< Written by the FRI thread, read by the consumer.*/
  StateInterfaceParameters parameters_;
  ExponentialFilterArray<N_JNTS> external_torque_filter_, measured_torque_filter_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__STATE_HPP_
