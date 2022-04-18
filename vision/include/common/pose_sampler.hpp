#ifndef COMMON_POSE_SAMPLER_HPP
#define COMMON_POSE_SAMPLER_HPP

#include <random>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace common {

class PoseSampler {

public:
  POINTER_TYPEDEF(PoseSampler);
  DISALLOW_EVIL_CONSTRUCTORS(PoseSampler);

  PoseSampler(
    Eigen::Affine3d const& mean,
    double sigma_orientation,
    double sigma_position);

  PoseSampler(
    Eigen::Affine3d const& mean,
    Eigen::Matrix<double,6,6> const& covariance);

public:

  void setMaxOrientationDeviation(double max_orientation_deviation);
  void setMaxPositionDeviation(double max_position_deviation);
  Eigen::Affine3d sample() const;

private:

  Eigen::Matrix<double,6,6> setCholeskyMatrix(double sigma_orientation, double sigma_position);
  Eigen::Matrix<double,6,6> setCholeskyMatrix(Eigen::Matrix<double,6,6> const& covariance);

private:
  Eigen::Affine3d mean_;
  Eigen::Matrix<double,6,6> cholesky_;
  double max_orientation_deviation_;
  double max_position_deviation_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable distribution_;

}; // class PoseSampler

} // namespace common

#endif // COMMON_POSE_SAMPLER_HPP
