#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {

LBRClient::LBRClient(const std::shared_ptr<lbr_fri_ros2::LBRIntermediary> lbr_intermediary)
    : lbr_intermediary_(lbr_intermediary){};

void LBRClient::onStateChange(KUKA::FRI::ESessionState old_state,
                              KUKA::FRI::ESessionState new_state) {
  printf("LBR switched from %s to %s.\n", session_state_to_string(old_state).c_str(),
         session_state_to_string(new_state).c_str());
}
void LBRClient::monitor() { lbr_intermediary_->state_to_buffer(robotState()); }

void LBRClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  lbr_intermediary_->state_to_buffer(robotState());
}

void LBRClient::command() {
  lbr_intermediary_->state_to_buffer(robotState());
  lbr_intermediary_->buffer_to_command(robotCommand());
}

std::string LBRClient::session_state_to_string(const KUKA::FRI::ESessionState &state) {
  switch (state) {
  case KUKA::FRI::ESessionState::IDLE:
    return "IDLE";
  case KUKA::FRI::ESessionState::MONITORING_WAIT:
    return "MONITORING_WAIT";
  case KUKA::FRI::ESessionState::MONITORING_READY:
    return "MONITORING_READY";
  case KUKA::FRI::ESessionState::COMMANDING_WAIT:
    return "COMMANDING_WAIT";
  case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
    return "COMMANDING_ACTIVE";
  default:
    throw std::runtime_error("Reveived unknown state.");
  }
}
} // end of namespace lbr_fri_ros2
