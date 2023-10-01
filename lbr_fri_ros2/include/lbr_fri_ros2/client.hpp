#ifndef LBR_FRI_ROS2__CLIENT_HPP_
#define LBR_FRI_ROS2__CLIENT_HPP_

#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_ros2/command_interface.hpp"
#include "lbr_fri_ros2/state_interface.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Simple implementation of KUKA::FRI::LBRClient. Has a shared node for reading commands
 * from / writing states to topics that follow the robot's QoS.
 *
 */
class Client : public KUKA::FRI::LBRClient {
public:
  Client() = delete;
  Client(const rclcpp::Node::SharedPtr node_ptr)
      : logging_interface_ptr_(node_ptr->get_node_logging_interface()),
        parameters_interface_ptr_(node_ptr->get_node_parameters_interface()),
        command_interface_(node_ptr),
        state_interface_(logging_interface_ptr_, parameters_interface_ptr_), open_loop_(true){};

  inline CommandInterface &get_command_interface() { return command_interface_; }
  inline StateInterface &get_state_interface() { return state_interface_; }

  /**
   * @brief Prints state change to terminal.
   *
   * @param[in] old_state The robot's old state
   * @param[in] new_state The robot's new state
   */
  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override {
    RCLCPP_INFO(logging_interface_ptr_->get_logger(), "LBR switched from %s to %s.",
                KUKA_FRI_STATE_MAP[old_state].c_str(), KUKA_FRI_STATE_MAP[new_state].c_str());
    command_interface_.init_command(robotState());
  }

  /**
   * @brief Called when robot in KUKA::FRI::MONITORING_WAIT or KUKA::FRI::MONITORING_READY state.
   * Publishes state from #robotState via #lbr_state_pub_.
   *
   */
  void monitor() override { state_interface_.set_state(robotState()); };

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_WAIT state. Publishes state from #robotState
   * via #lbr_state_pub_ and sets #robotCommand if in command mode KUKA::FRI::WRENCH or
   * KUKA::FRI::TORQUE.
   *
   */
  void waitForCommand() override {
    KUKA::FRI::LBRClient::waitForCommand();
    state_interface_.set_state(robotState());

    if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
      command_interface_.get_torque_command(robotCommand(), robotState());
    }

    if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::WRENCH) {
      command_interface_.get_wrench_command(robotCommand(), robotState());
    }
  };

  void command() override {
    if (open_loop_) {
      state_interface_.set_state_open_loop(robotState(),
                                           command_interface_.get_command().joint_position);
    } else {
      state_interface_.set_state(robotState());
    }

    switch (robotState().getClientCommandMode()) {
    case KUKA::FRI::EClientCommandMode::POSITION:
      command_interface_.get_joint_position_command(robotCommand(), robotState());
      return;
    case KUKA::FRI::EClientCommandMode::TORQUE:
      command_interface_.get_torque_command(robotCommand(), robotState());
      return;
    case KUKA::FRI::EClientCommandMode::WRENCH:
      command_interface_.get_wrench_command(robotCommand(), robotState());
      return;
    default:
      std::string err =
          "Unsupported command mode: " + std::to_string(robotState().getClientCommandMode()) + ".";
      RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
      throw std::runtime_error(err);
    }
  };

protected:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  CommandInterface command_interface_;
  StateInterface state_interface_;

  bool open_loop_;

private:
  std::map<int, std::string> KUKA_FRI_STATE_MAP{
      {KUKA::FRI::ESessionState::IDLE, "IDLE"},
      {KUKA::FRI::ESessionState::MONITORING_WAIT, "MONITORING_WAIT"},
      {KUKA::FRI::ESessionState::MONITORING_READY, "MONITORING_READY"},
      {KUKA::FRI::ESessionState::COMMANDING_WAIT, "COMMANDING_WAIT"},
      {KUKA::FRI::ESessionState::COMMANDING_ACTIVE, "COMMANDING_ACTIVE"},
  }; /** Map for converting KUKA::FRI::ESessionState to readable strings.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__CLIENT_HPP_
