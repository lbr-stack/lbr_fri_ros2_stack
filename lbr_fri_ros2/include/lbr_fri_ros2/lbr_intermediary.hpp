#ifndef LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
#define LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_

#include <memory>
#include <stdexcept>

#include "fri/FRIMessages.pb.h"
#include "friClientIf.h"
#include "friLBRCommand.h"
#include "friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr_command_guard.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief LBRIntermediary serves as an intermediary buffer for commands and states between ROS and
 * the FRI. Commands are checked for limits prior to writing them into the buffer.
 *
 */
class LBRIntermediary {
public:
  LBRIntermediary() = default;

  /**
   * @brief Construct a new LBRIntermediary object.
   *
   * @param[in] lbr_command_guard Command guard that is used to validate commands that the user
   * attempts to write to the #lbr_command_buffer_ via #command_to_buffer
   */
  LBRIntermediary(const lbr_fri_ros2::LBRCommandGuard &lbr_command_guard);

  /**
   * @brief Zeros the #lbr_command_buffer_. Sets the commanded joint position to the previously
   * commanded joint position. Zeros all commanded torques and wrenches.
   *
   * @param[in] lbr_state The current state. Includes the previous command
   * @return true if sucessful
   * @return false if unsecessful
   */
  bool zero_command_buffer(const KUKA::FRI::LBRState &lbr_state);

  /**
   * @brief Writes the desired command to the #lbr_command_buffer_ if valid (user to buffer).
   * Validates the desired command via #lbr_command_guard_.
   *
   * @param[in] lbr_command The desired command
   * @return true if desired command succesfully written to buffer
   * @return false if desired command invalid
   */
  bool command_to_buffer(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr lbr_command);

  /**
   * @brief Writes the #lbr_command_buffer_ to lbr_command (buffer to robot).
   *
   * If robot in
   * - KUKA::FRI::POSITION: Writes desired positions from #lbr_command_buffer_ to lbr_command.
   * - KUKA::FRI::WRENCH: Writes desired positions and wrenches from #lbr_command_buffer_ to
   * lbr_command.
   * - KUKA::FRI::TORQUE: Writes desired positions and torques from #lbr_command_buffer_ to
   * lbr_command.
   *
   * @param[out] lbr_command The command that is passed to the robot
   * @return true if #lbr_command_buffer_ successfully written to lbr_command
   * @return false if #lbr_command_buffer_ unsuccessfully written to lbr_command
   */
  bool buffer_to_command(KUKA::FRI::LBRCommand &lbr_command) const;

  /**
   * @brief Writes the lbr_state to the #lbr_state_buffer_ (robot to buffer).
   *
   * @param[in] lbr_state The current state
   * @return true if successful
   * @return false if unsuccessful
   */
  bool state_to_buffer(const KUKA::FRI::LBRState &lbr_state);

  /**
   * @brief Writes the #lbr_state_buffer_ to lbr_state (buffer to user).
   *
   * @param[out] lbr_state The state that is passed to the user
   * @return true if successful
   * @return false if unsuccessful
   */
  bool buffer_to_state(lbr_fri_msgs::msg::LBRState &lbr_state) const;

  /**
   * @brief Getter for #lbr_command_buffer_.
   *
   * @return const lbr_fri_msgs::msg::LBRCommand&
   */
  inline const lbr_fri_msgs::msg::LBRCommand &lbr_command() const { return lbr_command_buffer_; };

  /**
   * @brief Getter for #lbr_state_buffer_.
   *
   * @return const lbr_fri_msgs::msg::LBRState&
   */
  inline const lbr_fri_msgs::msg::LBRState &lbr_state() const { return lbr_state_buffer_; };

protected:
  std::unique_ptr<lbr_fri_ros2::LBRCommandGuard>
      lbr_command_guard_; /**< The command guard for validating commands in #buffer_to_command prior
                             to writing them to #lbr_command_buffer_.*/
  lbr_fri_msgs::msg::LBRCommand lbr_command_buffer_; /**< Buffer for the robot command.*/
  lbr_fri_msgs::msg::LBRState lbr_state_buffer_;     /**< Buffer for the robot state.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_INTERMEDIARY_HPP_
