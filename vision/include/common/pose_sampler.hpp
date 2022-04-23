#ifndef COMMON_POSE_SAMPLER_HPP
#define COMMON_POSE_SAMPLER_HPP

#include <random>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class PoseSampler {

public:
  POINTER_TYPEDEF(PoseSampler);
  DISALLOW_EVIL_CONSTRUCTORS(PoseSampler);

  PoseSampler(
    double sigma_w,
    double sigma_t);

  PoseSampler(
    Eigen::Affine3d const& mean,
    double sigma_w,
    double sigma_t);

  PoseSampler(
    Eigen::Affine3d const& mean,
    Eigen::Matrix<double,6,6> const& covariance);

public:

  void setMaxOrientationDeviation(double maxdev_w);
  void setMaxPositionDeviation(double maxdev_t);
  Eigen::Affine3d sample(Eigen::Affine3d const& T) const;
  inline Eigen::Affine3d sample() const;

public:

  static Eigen::Affine3d sample(
    Eigen::Affine3d const& T,
    double sigma_w, double sigma_t,
    double maxdev_w = std::numeric_limits<double>::infinity(),
    double maxdev_r = std::numeric_limits<double>::infinity());

  static Eigen::Affine3d sample(
    Eigen::Affine3d const&T,
    double sigma_wx, double sigma_wy, double sigma_wz,
    double sigma_tx, double sigma_ty, double sigma_tz,
    double maxdev_w = std::numeric_limits<double>::infinity(),
    double maxdev_t = std::numeric_limits<double>::infinity());

  static Eigen::Affine3d sample(
    Eigen::Affine3d const&T,
    double sigma_wx, double sigma_wy, double sigma_wz, 
    double sigma_tx, double sigma_ty, double sigma_tz,
    double maxdev_wx, double maxdev_wy, double maxdev_wz,
    double maxdev_tx, double maxdev_ty, double maxdev_tz);

private:

  Eigen::Matrix<double,6,6> setCholeskyMatrix(double sigma_w, double sigma_t);
  Eigen::Matrix<double,6,6> setCholeskyMatrix(Eigen::Matrix<double,6,6> const& covariance);

private:
  Eigen::Affine3d mean_;
  Eigen::Matrix<double,6,6> cholesky_;
  double maxdev_w_;
  double maxdev_t_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable distribution_;

}; // class PoseSampler

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseSampler::sample() const
{
  return sample(mean_);
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_POSE_SAMPLER_HPP
