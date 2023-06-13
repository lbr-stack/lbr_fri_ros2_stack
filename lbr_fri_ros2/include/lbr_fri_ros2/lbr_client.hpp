#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <map>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "fri/friClientIf.h"
#include "fri/friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr_command_guard.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief Simple implementation of KUKA::FRI::LBRClient. Allows for command injection and state
 * extraction via a shared lbr_fri_ros2::LBRIntermediary object.
 *
 */
class LBRClient : public KUKA::FRI::LBRClient {
public:
  LBRClient() = delete;

  /**
   * @brief Construct a new LBRClient object.
   *
   * @param[in] lbr_intermediary Shared pointer to lbr_fri_ros2::LBRIntermediary for command
   * injection and state extraction.
   */
  LBRClient(const rclcpp::Node::SharedPtr node, const lbr_fri_ros2::LBRCommandGuard& lbr_command_guard);

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
   * KUKA::FRI::MONITORING_READY state. Writes state from #robotState to
   * #lbr_intermediary_.
   *
   */
  void monitor() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_WAIT state. Writes state from
   * #robotState to #lbr_intermediary_ and sets #robotCommand to commanded
   * state via LBRClient#zero_command_.
   *
   */
  void waitForCommand() override;

  /**
   * @brief Called when robot in KUKA::FRI::COMMANDING_ACTIVE state. Writes state
   * from #robotState to #lbr_intermediary_. Writes command from #lbr_intermediary_ to
   * #robotCommand.
   *
   */
  void command() override;

protected:
  void pub_lbr_state_();
  void lbr_command_sub_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);
  void init_lbr_command_rt_buf_();

  rclcpp::Node::SharedPtr node_; /**< Shared pointer to node.*/

  std::unique_ptr<lbr_fri_ros2::LBRCommandGuard> lbr_command_guard_;

  std::shared_ptr<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand::SharedPtr>>
      lbr_command_rt_buf_; /**< Realtime-safe buffer for receiving lbr_fri_msgs::msg::LBRCommand
                              commands.*/
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRCommand>::SharedPtr
      lbr_command_sub_; /**< Subscribtion to lbr_fri_msgs::msg::LBRCommand commands.*/
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr
      lbr_state_pub_; /**< Publisher of lbr_fri_msgs::msg::LBRState.*/
  std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>
      lbr_state_rt_pub_; /**< Realtime-safe publisher of lbr_fri_msgs::msg::LBRState.*/

private:
  std::map<int, std::string> KUKA_FRI_STATE_MAP{
      {KUKA::FRI::ESessionState::IDLE, "IDLE"},
      {KUKA::FRI::ESessionState::MONITORING_WAIT, "MONITORING_WAIT"},
      {KUKA::FRI::ESessionState::MONITORING_READY, "MONITORING_READY"},
      {KUKA::FRI::ESessionState::COMMANDING_WAIT, "COMMANDING_WAIT"},
      {KUKA::FRI::ESessionState::COMMANDING_ACTIVE, "COMMANDING_ACTIVE"},
  };
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_CLIENT_HPP_
