#ifndef LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "friLBRClient.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief CommandGuard checks desired commands for limits.
 *
 */
class CommandGuard {
protected:
  using JointArray = lbr_fri_msgs::msg::LBRCommand::_joint_position_type;

public:
  CommandGuard() = delete;

  /**
   * @brief Construct a new CommandGuard object.
   *
   * @param[in] logger_interface Shared node logger interface
   * @param[in] min_position Minimum joint position [rad]
   * @param[in] max_position Maximum joint position [rad]
   * @param[in] max_velocity Maximum joint velocity [rad/s]
   * @param[in] max_torque Maximal torque [Nm]
   */
  CommandGuard(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
               const JointArray &min_position, const JointArray &max_position,
               const JointArray &max_velocity, const JointArray &max_torque);

  /**
   * @brief Construct a new CommandGuard object.
   *
   * @param[in] logger_interface Shared node logger interface
   * @param robot_description String containing URDF robot rescription
   */
  CommandGuard(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
               const std::string &robot_description);

  /**
   * @brief Checks for command validity given the current state.
   *
   * @param[in] lbr_command The desired command
   * @param[in] lbr_state The current state
   * @return true if lbr_command is valid
   * @return false if lbr_command is invalid
   */
  virtual bool is_valid_command(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                const KUKA::FRI::LBRState &lbr_state) const;

protected:
  /**
   * @brief Checks for nans.
   *
   * @param[in] begin Pointer to begin
   * @param[in] end Pointer to end
   * @return true if any nans between begin to end
   * @return false if no nans between begin to end
   */
  bool is_nan_(const double *begin, const double *end) const;

  /**
   * @brief Checks for joint position limits.
   *
   * @param[in] lbr_command The desired command
   * @param[in] lbr_state The current state
   * @return true if lbr_command in position limits
   * @return false if lbr_command outside position limits
   */
  virtual bool command_in_position_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                           const KUKA::FRI::LBRState & /*lbr_state*/) const;

  /**
   * @brief Checks for joint velocity limits.
   *
   * @param[in] lbr_command The desired command
   * @param[in] lbr_state The current state
   * @return true if lbr_command in velocity limits
   * @return false if lbr_command outside velocity limits
   */
  virtual bool command_in_velocity_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                           const KUKA::FRI::LBRState &lbr_state) const;

  /**
   * @brief Checks for joint torque limits.
   *
   * @param[in] lbr_command The desired command
   * @param[in] lbr_state The current state
   * @return true if lbr_command in torque limits
   * @return false if lbr_command outside torque limits
   */
  virtual bool command_in_torque_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                         const KUKA::FRI::LBRState &lbr_state) const;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
      logger_interface_; /**< Shared node logger interface.*/

  JointArray min_position_; /**< Minimum joint position [rad].*/
  JointArray max_position_; /**< Maximum joint position [rad].*/
  JointArray max_velocity_; /**< Maximum joint velocity [rad/s].*/
  JointArray max_torque_;   /**< Maximum joint torque [Nm].*/
};

/**
 * @brief Adds early stopping to CommandGuard.
 *
 */
class SafeStopCommandGuard : public CommandGuard {
public:
  SafeStopCommandGuard() = delete;

  /**
   * @brief Construct a new SafeStopCommandGuard object.
   *
   * @param[in] logger_interface Shared node logger interface
   * @param robot_description String containing URDF robot rescription
   */
  SafeStopCommandGuard(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
      const std::string &robot_description)
      : CommandGuard(logger_interface, robot_description){};

protected:
  /**
   * @brief Checks for joint position limits and guarantees an early stop given the maximum
   * velocity.
   *
   * @param[in] lbr_command The desired command
   * @param[in] lbr_state The current state
   * @return true if lbr_command in position limits
   * @return false if lbr_command outside position limits
   */
  virtual bool command_in_position_limits_(const lbr_fri_msgs::msg::LBRCommand &lbr_command,
                                           const KUKA::FRI::LBRState &lbr_state) const override;
};

/**
 * @brief Creates an CommandGuard object.
 *
 * @param[in] logger_interface Shared node logger interface
 * @param[in] robot_description String containing URDF robot rescription
 * @param[in] variant Which variant of CommandGuard to create
 * @return std::unique_ptr<CommandGuard> Pointer to CommandGuard object
 */
std::unique_ptr<CommandGuard> lbr_command_guard_factory(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
    const std::string &robot_description, const std::string &variant);
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_COMMAND_GUARD_HPP_
