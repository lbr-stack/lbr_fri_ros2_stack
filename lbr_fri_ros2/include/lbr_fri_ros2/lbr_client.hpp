#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <cstring>
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
 * @brief Simple implementation of KUKA::FRI::LBRClient. Has a shared node for reading commands
 * from / writing states to topics that follow the robot's QoS.
 *
 */
class LBRClient : public KUKA::FRI::LBRClient {
public:
  LBRClient() = delete;

  /**
   * @brief Construct a new LBRClient object.
   *
   * @param[in] node Shared node for reading commands from / writing states to topics.
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
   * @brief Called when robot in KUKA::FRI::MONITORING_WAIT or KUKA::FRI::MONITORING_READY state.
   * Publishes state from #robotState via #lbr_state_pub_.
   *
   */
  void monitor() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_WAIT state. Publishes state from #robotState
   * via #lbr_state_pub_ and sets #robotCommand if in command mode KUKA::FRI::WRENCH or
   * KUKA::FRI::TORQUE.
   *
   */
  void waitForCommand() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_ACTIVE state. Publishes state from
   * #robotState via #lbr_state_pub_. Writes command from #lbr_command_ to #robotCommand.
   *
   */
  void command() override;

protected:
  /**
   * @brief Set the the #lbr_command_ to the current robot configuration. Sets commanded wrenches
   * and torques to zero.
   *
   */
  void init_lbr_command_();

  /**
   * @brief Initialize topics, that is #lbr_command_sub_ and #lbr_state_pub_. This method may only
   * be called after #robotState is accessible since QoS profiles utilize the robots's sample time.
   *
   */
  void init_topics_();

  /**
   * @brief Declare parameters for #node_ that are utilized within LBRClient.
   *
   */
  void declare_parameters_();

  /**
   * @brief Get parameters from #node_ and write them to members.
   *
   */
  void get_parameters_();

  /**
   * @brief Publish #lbr_state_ using #lbr_state_pub_.
   *
   */
  void pub_lbr_state_();

  /**
   * @brief Callback for #lbr_command_sub_. Validates incoming commands using #lbr_command_guard_
   * prior to writing them into #lbr_command_.
   *
   * @param[in] lbr_command
   */
  void on_lbr_command_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);

  rclcpp::Node::SharedPtr node_; /**< Shared pointer to node.*/

  uint32_t missed_deadlines_pub_; /**< Counter for states that were not published within specified
                                     deadline.*/
  uint32_t missed_deadlines_sub_; /**< Counter for commands that were not received within specified
                                     deadline.*/

  lbr_fri_msgs::msg::LBRCommand lbr_command_; /**< Command buffer.*/
  lbr_fri_msgs::msg::LBRState lbr_state_;     /**< State buffer.*/

  std::string
      lbr_command_topic_; /**< Command topic to be subscribed from, defaults to /lbr_command.*/
  std::string lbr_state_topic_; /**< State topic to be published to, defaults to /lbr_state.*/
  double smoothing_;            /**< Exponential smoothing factor for position commands.*/

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
