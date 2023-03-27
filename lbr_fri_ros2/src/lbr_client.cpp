#include "lbr_fri_ros2/lbr_client.hpp"

namespace lbr_fri_ros2 {
LBRClient::LBRClient(const std::shared_ptr<lbr_fri_ros2::LBRIntermediary> lbr_intermediary)
    : lbr_intermediary_(lbr_intermediary){};

void LBRClient::onStateChange(KUKA::FRI::ESessionState old_state,
                              KUKA::FRI::ESessionState new_state) {
  printf("LBR switched from %s to %s.\n", session_state_to_string_(old_state).c_str(),
         session_state_to_string_(new_state).c_str());
}
void LBRClient::monitor() { state_to_buffer_(); }

void LBRClient::waitForCommand() {
  KUKA::FRI::LBRClient::waitForCommand();
  state_to_buffer_();
  zero_command_();
}

void LBRClient::command() {
  state_to_buffer_();
  buffer_to_command_();
}

void LBRClient::zero_command_() {
  if (!lbr_intermediary_->zero_command_buffer(robotState())) {
    throw std::runtime_error("Failed to zero the command.");
  }
}

void LBRClient::buffer_to_command_() {
  if (!lbr_intermediary_->buffer_to_command(robotCommand())) {
    throw std::runtime_error("Failed to write buffer to command.");
  }
}

void LBRClient::state_to_buffer_() {
  if (!lbr_intermediary_->state_to_buffer(robotState())) {
    throw std::runtime_error("Failed to write state to buffer.");
  }
}

std::string LBRClient::session_state_to_string_(const KUKA::FRI::ESessionState &state) {
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
