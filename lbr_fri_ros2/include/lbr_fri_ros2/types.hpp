#ifndef LBR_FRI_ROS2__TYPES_HPP_
#define LBR_FRI_ROS2__TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"

namespace lbr_fri_ros2 {
// joint positions, velocities, accelerations, torques etc.
using jnt_array_t = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
using jnt_array_t_ref = jnt_array_t &;
using const_jnt_array_t_ref = const jnt_array_t &;

// joint names
using jnt_name_array_t = std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
using jnt_name_array_t_ref = jnt_name_array_t &;
using const_jnt_name_array_t_ref = const jnt_name_array_t &;

// cartesian dof
constexpr std::uint8_t CARTESIAN_DOF = 6;

// cartesian positions, velocities, accelerations, torques etc.
using cart_array_t = std::array<double, CARTESIAN_DOF>;
using cart_array_t_ref = cart_array_t &;
using const_cart_array_t_ref = const cart_array_t &;

// idl types
using idl_command_t = lbr_fri_idl::msg::LBRCommand;
using idl_command_t_ref = idl_command_t &;
using const_idl_command_t_ref = const idl_command_t &;
using idl_state_t = lbr_fri_idl::msg::LBRState;
using idl_state_t_ref = idl_state_t &;
using const_idl_state_t_ref = const idl_state_t &;
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__TYPES_HPP_
