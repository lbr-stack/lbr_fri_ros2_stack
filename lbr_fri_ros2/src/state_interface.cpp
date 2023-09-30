#include "lbr_fri_ros2/state_interface.hpp"

namespace lbr_fri_ros2 {
void StateInteface::set_state(const_fri_state_type_ref state) {
  state_.client_command_mode = state.getClientCommandMode();
  std::memcpy(state_.commanded_joint_position.data(), state.getCommandedJointPosition(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  std::memcpy(state_.commanded_torque.data(), state.getCommandedTorque(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  state_.connection_quality = state.getConnectionQuality();
  state_.control_mode = state.getControlMode();
  state_.drive_state = state.getDriveState();
  // external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque); TODO:
  // update
  if (state.getSessionState() == fri_session_state_type::COMMANDING_WAIT ||
      state.getSessionState() == fri_session_state_type::COMMANDING_ACTIVE) {
    std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  }
  std::memcpy(state_.measured_joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  // measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();
};

void StateInteface::set_state_open_loop(const_fri_state_type_ref state,
                                        const_ros_joint_pos_type_ref joint_position) {
  state_.client_command_mode = state.getClientCommandMode();
  std::memcpy(state_.commanded_joint_position.data(), state.getCommandedJointPosition(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  std::memcpy(state_.commanded_torque.data(), state.getCommandedTorque(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  state_.connection_quality = state.getConnectionQuality();
  state_.control_mode = state.getControlMode();
  state_.drive_state = state.getDriveState();
  // external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);
  if (state.getSessionState() == fri_session_state_type::COMMANDING_WAIT ||
      state.getSessionState() == fri_session_state_type::COMMANDING_ACTIVE) {
    std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  }
  std::memcpy(state_.measured_joint_position.data(), joint_position.data(),
              sizeof(double) * fri_state_type::NUMBER_OF_JOINTS);
  // measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();
}
} // namespace lbr_fri_ros2