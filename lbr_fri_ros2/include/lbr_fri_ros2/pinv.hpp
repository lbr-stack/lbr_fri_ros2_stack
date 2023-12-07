#ifndef LBR_FRI_ROS2__PINV_HPP_
#define LBR_FRI_ROS2__PINV_HPP_

#include <algorithm>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"

namespace lbr_fri_ros2 {
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat,
     typename MatT::Scalar lambda = typename MatT::Scalar{2e-1}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  std::for_each(singularValues.data(), singularValues.data() + singularValues.size(),
                [&, i = 0](const Scalar &s) mutable {
                  dampedSingularValuesInv(i, i) = s / (s * s + lambda * lambda);
                  ++i;
                });
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
}
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__PINV_HPP_
