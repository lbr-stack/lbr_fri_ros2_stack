#ifndef LBR_FRI_ROS2__CLIENT_HPP_
#define LBR_FRI_ROS2__CLIENT_HPP_

#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "control_toolbox/pid_ros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_position_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_msgs/msg/lbr_torque_command.hpp"
#include "lbr_fri_msgs/msg/lbr_wrench_command.hpp"
#include "lbr_fri_ros2/command_interfaces.hpp"
#include "lbr_fri_ros2/state_interface.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Simple implementation of KUKA::FRI::LBRClient. Has a shared node for reading commands
 * from / writing states to topics that follow the robot's QoS.
 *
 */
template <typename command_interface_type> class Client : public KUKA::FRI::LBRClient {
protected:
  using command_interface_type_ref = command_interface_type &;
  using state_interface_type = StateInteface;
  using state_interface_type_ref = state_interface_type &;

public:
  Client() = delete;

  /**
   * @brief Construct a new Client object.
   *
   * @param[in] node Shared node for reading commands from / writing states to topics.
   *
   */
  Client(const rclcpp::Node::SharedPtr node)
      : Client(node->get_node_logging_interface(), node->get_node_parameters_interface()){};

  Client(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
         const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface)
      : logging_interface_ptr_(logging_interface),
        parameters_interface_ptr_(parameters_interface){};

  inline command_interface_type_ref get_command_interface() { return command_interface_; }
  inline state_interface_type_ref get_state_interface() { return state_interface_; }

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
  };
  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_ACTIVE state. Publishes state from
   * #robotState via #lbr_state_pub_. Writes command from #lbr_command_ to #robotCommand.
   *
   */
  void command() override {
    if (open_loop_) {
      state_interface_.set_state(robotState());
    } else {
      // state_interface_.set_state_open_loop(robotState(), ); TODO
    }
    // set command
    robotCommand().setJointPosition(robotState().getMeasuredJointPosition()); // TODO: replace this
  };

protected:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  command_interface_type command_interface_;
  state_interface_type state_interface_;

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
