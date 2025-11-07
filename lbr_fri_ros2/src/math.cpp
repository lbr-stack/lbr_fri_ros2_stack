#include "lbr_fri_ros2/math.hpp"

namespace lbr_fri_ros2 {
bool norm_in_bounds(const double &x0, const double &x1, const double &x2, const double &max) {
  double norm_sq = x0 * x0 + x1 * x1 + x2 * x2;
  return norm_sq <= max * max;
}
bool norm_in_bounds(const std::array<double, 3> &vec, const double &max) {
  return norm_in_bounds(vec[0], vec[1], vec[2], max);
}
} // namespace lbr_fri_ros2
