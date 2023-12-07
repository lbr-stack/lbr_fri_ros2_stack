#ifndef LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
#define LBR_FRI_ROS2__ASYNC_CLIENT_HPP_

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friLBRClient.h"

#include "lbr_fri_ros2/command_interface.hpp"
#include "lbr_fri_ros2/enum_maps.hpp"
#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/state_interface.hpp"

namespace lbr_fri_ros2 {
class AsyncClient : public KUKA::FRI::LBRClient {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::AsyncClient";

public:
  AsyncClient() = delete;
  AsyncClient(const PIDParameters &pid_parameters,
              const CommandGuardParameters &command_guard_parameters,
              const std::string &command_guard_variant,
              const StateInterfaceParameters &state_interface_parameters = {10.0, 10.0},
              const bool &open_loop = true);

  inline CommandInterface &get_command_interface() { return command_interface_; }
  inline StateInterface &get_state_interface() { return state_interface_; }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

protected:
  CommandInterface command_interface_;
  StateInterface state_interface_;

  bool open_loop_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
