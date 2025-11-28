#include "lbr_fri_ros2/math.hpp"

namespace lbr_fri_ros2 {
bool norm_in_bounds(const double &x0, const double &x1, const double &x2, const double &max) {
  double norm_sq = x0 * x0 + x1 * x1 + x2 * x2;
  return norm_sq <= max * max;
}
bool norm_in_bounds(const std::array<double, 3> &vec, const double &max) {
  return norm_in_bounds(vec[0], vec[1], vec[2], max);
}
bool all_jnts_in_bounds(const_jnt_array_t_ref vals, const_jnt_array_t_ref lower,
                        const_jnt_array_t_ref upper) {
  for (std::size_t i = 0; i < vals.size(); ++i) {
    if (vals[i] < lower[i] || upper[i] < vals[i]) {
      return false;
    }
  }
  return true;
}
} // namespace lbr_fri_ros2
