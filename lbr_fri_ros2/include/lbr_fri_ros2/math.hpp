#ifndef LBR_FRI_ROS2__MATH_HPP_
#define LBR_FRI_ROS2__MATH_HPP_

#include <array>

namespace lbr_fri_ros2 {
bool norm_in_bounds(const double &x0, const double &x1, const double &x2, const double &max);
bool norm_in_bounds(const std::array<double, 3> &vec, const double &max);
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__MATH_HPP_
