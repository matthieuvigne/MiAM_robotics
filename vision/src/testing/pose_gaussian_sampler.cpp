#include <common/maths.hpp>
#include <testing/pose_gaussian_sampler.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

PoseGaussianSampler::PoseGaussianSampler(
  Eigen::Affine3d const& mean,
  double sigma_w, double sigma_t)
: PoseSamplerBase (Type::GAUSSIAN),
  mean_           (mean),
  cholesky_       (setCholeskyMatrix(sigma_w, sigma_t)),
  maxdev_w_       (std::numeric_limits<double>::max()),
  maxdev_t_       (std::numeric_limits<double>::max()),
  distribution_   (0.0, 1.0)
{
  CHECK(sigma_w >= 0);
  CHECK(sigma_t >= 0);
}

//--------------------------------------------------------------------------------------------------

PoseGaussianSampler::PoseGaussianSampler(
  double sigma_w, double sigma_t)
: PoseGaussianSampler(Eigen::Affine3d::Identity(), sigma_w, sigma_t)
{}

//--------------------------------------------------------------------------------------------------

PoseGaussianSampler::PoseGaussianSampler(
  Eigen::Affine3d const& mean,
  Eigen::Matrix<double,6,6> const& covariance)
: PoseSamplerBase (Type::GAUSSIAN),
  mean_           (mean),
  cholesky_       (setCholeskyMatrix(covariance)),
  maxdev_w_       (std::numeric_limits<double>::max()),
  maxdev_t_       (std::numeric_limits<double>::max()),
  distribution_   (0.0, 1.0)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseGaussianSampler::sample() const
{
  // Sample
  Eigen::Matrix<double,6,1> tau;
  for(int i=0; i<6; i++) tau(i) = distribution_(generator_);
  tau = cholesky_ * tau;

  // Bound the sampled deviation
  for(int i=0; i<3; i++) // <- Bound the orientation error
    tau(i) = std::max(-maxdev_w_, std::min(tau(i), maxdev_w_));
  for(int i=3; i<6; i++) // <- Bound the position error
    tau(i) = std::max(-maxdev_t_, std::min(tau(i), maxdev_t_));
  return common::so3r3::product(tau,mean_);
}

//--------------------------------------------------------------------------------------------------

void PoseGaussianSampler::setMaxOrientationDeviation(double maxdev_w)
{
  CHECK(maxdev_w >= 0.);
  maxdev_w_ = maxdev_w;
}

//--------------------------------------------------------------------------------------------------

void PoseGaussianSampler::setMaxPositionDeviation(double maxdev_t)
{
  CHECK(maxdev_t >= 0.);
  maxdev_t_ = maxdev_t;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> PoseGaussianSampler::setCholeskyMatrix(
  double sigma_w, double sigma_t)
{
  Eigen::Matrix<double,6,6> cholesky = Eigen::Matrix<double,6,6>::Identity();
  cholesky.block<3,3>(0,0) *= sigma_w;
  cholesky.block<3,3>(3,3) *= sigma_t;
  return cholesky;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> PoseGaussianSampler::setCholeskyMatrix(
  Eigen::Matrix<double,6,6> const& covariance)
{
  return Eigen::Matrix<double,6,6>(covariance.llt().matrixL());
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseGaussianSampler::sample(
  Eigen::Affine3d const& T,
  double sigma_w, double sigma_t,
  double maxdev_w, double maxdev_t)
{
  return sample(T, sigma_w, sigma_w, sigma_w,
                   sigma_t, sigma_t, sigma_t,
                   maxdev_w, maxdev_w, maxdev_w,
                   maxdev_t, maxdev_t, maxdev_t);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseGaussianSampler::sample(
  Eigen::Affine3d const&T,
  double sigma_wx, double sigma_wy, double sigma_wz,
  double sigma_tx, double sigma_ty, double sigma_tz,
  double maxdev_w, double maxdev_t)
{
  return sample(T, sigma_wx, sigma_wy, sigma_wz,
                   sigma_tx, sigma_ty, sigma_tz,
                   maxdev_w, maxdev_w, maxdev_w,
                   maxdev_t, maxdev_t, maxdev_t);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseGaussianSampler::sample(
  Eigen::Affine3d const&T,
  double sigma_wx, double sigma_wy, double sigma_wz, 
  double sigma_tx, double sigma_ty, double sigma_tz,
  double maxdev_wx, double maxdev_wy, double maxdev_wz,
  double maxdev_tx, double maxdev_ty, double maxdev_tz)
{
  // Initialize static Gaussian samplers
  static std::default_random_engine generator;
  static std::normal_distribution<double> N(0.0,1.0);
  CHECK( maxdev_wx > 0 ); CHECK( maxdev_wy > 0 ); CHECK( maxdev_wz > 0 );
  CHECK( maxdev_tx > 0 ); CHECK( maxdev_ty > 0 ); CHECK( maxdev_tz > 0 );

  // Sample the orientation and translation increment
  Eigen::Matrix<double,6,1> tau;
  enum { WX, WY, WZ, TX, TY, TZ };
  tau(WX) = std::max( -maxdev_wx, std::min( sigma_wx * N(generator), maxdev_wx ) );
  tau(WY) = std::max( -maxdev_wy, std::min( sigma_wy * N(generator), maxdev_wy ) );
  tau(WZ) = std::max( -maxdev_wz, std::min( sigma_wz * N(generator), maxdev_wz ) );
  tau(TX) = std::max( -maxdev_tx, std::min( sigma_tx * N(generator), maxdev_tx ) );
  tau(TY) = std::max( -maxdev_ty, std::min( sigma_ty * N(generator), maxdev_ty ) );
  tau(TZ) = std::max( -maxdev_tz, std::min( sigma_tz * N(generator), maxdev_tz ) );
  return common::so3r3::product(tau, T);
}

//--------------------------------------------------------------------------------------------------

} // namespace testing
