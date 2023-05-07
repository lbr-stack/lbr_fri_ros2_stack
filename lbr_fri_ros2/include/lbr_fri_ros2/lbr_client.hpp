#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <memory>
#include <stdexcept>

#include "friLBRClient.h"

#include "lbr_fri_ros2/lbr_intermediary.hpp"

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
   * @param[in] lbr_intermediary Shared pointer to lbr_fri_ros2::LBRIntermediary for command injection
   * and state extraction.
   */
  LBRClient(const std::shared_ptr<lbr_fri_ros2::LBRIntermediary> lbr_intermediary);

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
  /**
   * @brief Sets command to commanded state.
   *
   */
  void zero_command_();

  /**
   * @brief Writes command from #lbr_intermediary_ to #robotState.
   *
   */
  void buffer_to_command_();

  /**
   * @brief Writes state from #robotState to #lbr_intermediary_.
   *
   */
  void state_to_buffer_();

  std::string
  session_state_to_string_(const KUKA::FRI::ESessionState
                               &state); /**< Turns KUKA::FRI::ESessionState into readable string.*/

  const std::shared_ptr<lbr_fri_ros2::LBRIntermediary>
      lbr_intermediary_; /**< Command injection and state extraction through this intermediary
                            object.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_CLIENT_HPP_
