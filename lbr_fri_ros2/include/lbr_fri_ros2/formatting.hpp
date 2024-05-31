#ifndef LBR_FRI_ROS2__FORMATTING_HPP_
#define LBR_FRI_ROS2__FORMATTING_HPP_

#include <string>

#include "friLBRClient.h"
#include "friVersion.h"

namespace lbr_fri_ros2 {
struct ColorScheme {
  // refer https://stackoverflow.com/a/287944
  static constexpr char HEADER[] = "\033[95m";
  static constexpr char OKBLUE[] = "\033[94m";
  static constexpr char OKCYAN[] = "\033[96m";
  static constexpr char OKGREEN[] = "\033[92m";
  static constexpr char WARNING[] = "\033[93m";
  static constexpr char ERROR[] = "\033[91m";
  static constexpr char ENDC[] = "\033[0m";
  static constexpr char BOLD[] = "\033[1m";
  static constexpr char UNDERLINE[] = "\033[4m";
};

struct EnumMaps {
  static std::string session_state_map(const int &session_state) {
    switch (session_state) {
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
      return "UNKNOWN";
    }
  };

  static std::string control_mode_map(const int &control_mode) {
    switch (control_mode) {
    case KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE:
      return "CART_IMP_CONTROL_MODE";
    case KUKA::FRI::EControlMode::JOINT_IMP_CONTROL_MODE:
      return "JOINT_IMP_CONTROL_MODE";
    case KUKA::FRI::EControlMode::NO_CONTROL:
      return "NO_CONTROL";
    case KUKA::FRI::EControlMode::POSITION_CONTROL_MODE:
      return "POSITION_CONTROL_MODE";
    default:
      return "UNKNOWN";
    }
  };

  static std::string client_command_mode_map(const int &client_command_mode) {
    switch (client_command_mode) {
    case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
      return "NO_COMMAND_MODE";
#if FRICLIENT_VERSION_MAJOR == 1
    case KUKA::FRI::EClientCommandMode::POSITION:
      return "POSITION";
#endif
#if FRICLIENT_VERSION_MAJOR >= 2
    case KUKA::FRI::EClientCommandMode::JOINT_POSITION:
      return "JOINT_POSITION";
    case KUKA::FRI::EClientCommandMode::CARTESIAN_POSE:
      return "CARTESIAN_POSE";
#endif
    case KUKA::FRI::EClientCommandMode::TORQUE:
      return "TORQUE";
    case KUKA::FRI::EClientCommandMode::WRENCH:
      return "WRENCH";
    default:
      return "UNKNOWN";
    }
  };
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__ENUM_MAPS_HPP
