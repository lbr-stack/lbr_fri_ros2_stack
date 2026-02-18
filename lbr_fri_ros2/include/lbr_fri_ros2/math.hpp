#ifndef LBR_FRI_ROS2__MATH_HPP_
#define LBR_FRI_ROS2__MATH_HPP_

#include <array>

#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
bool norm_in_bounds(const double &x0, const double &x1, const double &x2, const double &max);
bool norm_in_bounds(const std::array<double, 3> &vec, const double &max);
bool all_jnts_in_bounds(const_jnt_array_t_ref vals, const_jnt_array_t_ref lower,
                        const_jnt_array_t_ref upper);
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__MATH_HPP_
