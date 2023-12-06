#ifndef LBR_FRI_ROS2__DAMPED_LEAST_SQUARES_HPP_
#define LBR_FRI_ROS2__DAMPED_LEAST_SQUARES_HPP_

#include <Eigen/Core>
#include <Eigen/SVD>

namespace lbr_fri_ros2 {
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
damped_least_squares(const MatT &mat, typename MatT::Scalar lambda =
                                          typename MatT::Scalar{2e-1}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    dampedSingularValuesInv(i, i) =
        singularValues(i) / (singularValues(i) * singularValues(i) + lambda * lambda);
  }
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
}
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__DAMPED_LEAST_SQUARES_HPP_
