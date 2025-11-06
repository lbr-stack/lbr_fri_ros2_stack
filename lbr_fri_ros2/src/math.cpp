#include "lbr_fri_ros2/math.hpp"

namespace lbr_fri_ros2 {
bool norm_in_bounds(const std::array<double, 3> &vec, const double &max) {
  double norm_sq = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
  return norm_sq <= max * max;
}
} // namespace lbr_fri_ros2
