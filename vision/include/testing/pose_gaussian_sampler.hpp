#ifndef TESTING_POSE_GAUSSIAN_SAMPLER_HPP
#define TESTING_POSE_GAUSSIAN_SAMPLER_HPP

#include <testing/pose_sampler_base.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class PoseGaussianSampler : public PoseSamplerBase {

public:
  POINTER_TYPEDEF(PoseGaussianSampler);
  DISALLOW_EVIL_CONSTRUCTORS(PoseGaussianSampler);
  PoseGaussianSampler(double sigma_w, double sigma_t);
  PoseGaussianSampler(Eigen::Affine3d const& mean, double sigma_w, double sigma_t);
  PoseGaussianSampler(Eigen::Affine3d const& mean, Eigen::Matrix<double,6,6> const& covariance);

public:

  void setMaxOrientationDeviation(double maxdev_w);
  void setMaxPositionDeviation(double maxdev_t);
  Eigen::Affine3d sample() const;

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

  Eigen::Affine3d const& mean_;
  double maxdev_w_;
  double maxdev_t_;
  Eigen::Matrix<double,6,6> cholesky_;
  std::normal_distribution<double> mutable distribution_;

}; // class PoseGaussianSampler

//--------------------------------------------------------------------------------------------------

} // namespace testing

#endif // TESTING_POSE_GAUSSIAN_SAMPLER_HPP
