#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "control_toolbox/filters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"

#include "fri/friClientIf.h"
#include "fri/friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr_command_guard.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Simple implementation of KUKA::FRI::LBRClient. Has a shared node for reading commands from
 * / writing states to real-time safe topics.
 *
 */
class LBRClient : public KUKA::FRI::LBRClient {
public:
  LBRClient() = delete;

  /**
   * @brief Construct a new LBRClient object.
   *
   * @param[in] node Shared node for reading commands from / writing states to real-time safe
   * topics.
   * @param[in] lbr_command_guard Command guard for validating incoming commands.
   *
   */
  LBRClient(const rclcpp::Node::SharedPtr node, std::unique_ptr<LBRCommandGuard> lbr_command_guard);

  /**
   * @brief Log the status of the robot to terminal.
   *
   */
  void log_status();

  /**
   * @brief Prints state change to terminal.
   *
   * @param[in] old_state The robot's old state
   * @param[in] new_state The robot's new state
   */
  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;

  /**
   * @brief Called when robot in KUKA::FRI::MONITORING_WAIT and
   * KUKA::FRI::MONITORING_READY state. Publishes state from #robotState via #lbr_state_rt_pub_.
   *
   */
  void monitor() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_WAIT state. Publishes state from #robotState
   * via #lbr_state_rt_pub_ and sets #robotCommand if in command mode KUKA::FRI::WRENCH or
   * KUKA::FRI::TORQUE.
   *
   */
  void waitForCommand() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_ACTIVE state. Publishes state from
   * #robotState via #lbr_state_rt_pub_. Writes command from #lbr_command_rt_buf_ to #robotCommand.
   *
   */
  void command() override;

protected:
  void init_lbr_command_();
  void init_topics_();

  void declare_parameters_();
  void get_parameters_();

  void pub_lbr_state_();
  void on_lbr_command_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);

  rclcpp::Node::SharedPtr node_; /**< Shared pointer to node.*/

  uint32_t missed_deadlines_pub_, missed_deadlines_sub_;

  lbr_fri_msgs::msg::LBRCommand lbr_command_;
  lbr_fri_msgs::msg::LBRState lbr_state_;

  std::string lbr_command_topic_, lbr_state_topic_;
  double smoothing_;

  std::unique_ptr<LBRCommandGuard>
      lbr_command_guard_; /**< Validating commands prior to writing them to #robotCommand.*/

  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr
      lbr_state_pub_; /**< Publisher of lbr_fri_msgs::msg::LBRState.*/
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRCommand>::SharedPtr
      lbr_command_sub_; /**< Subscribtion to lbr_fri_msgs::msg::LBRCommand commands.*/

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
#endif // LBR_FRI_ROS2__LBR_CLIENT_HPP_
