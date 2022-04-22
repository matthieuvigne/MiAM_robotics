#include <common/maths.hpp>
#include <common/pose_sampler.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

PoseSampler::PoseSampler(
  Eigen::Affine3d const& mean,
  double sigma_orientation,
  double sigma_position)
: mean_                       (mean),
  cholesky_                   (setCholeskyMatrix(sigma_orientation, sigma_position)),
  max_orientation_deviation_  (std::numeric_limits<double>::max()),
  max_position_deviation_     (std::numeric_limits<double>::max()),
  generator_                  (),
  distribution_               (0.0, 1.0)
{
  CHECK(sigma_orientation > 0);
  CHECK(sigma_position    > 0);
}

//--------------------------------------------------------------------------------------------------

PoseSampler::PoseSampler(
  double sigma_orientation,
  double sigma_position)
: PoseSampler(Eigen::Affine3d::Identity(), sigma_orientation, sigma_position)
{}

//--------------------------------------------------------------------------------------------------

PoseSampler::PoseSampler(
  Eigen::Affine3d const& mean,
  Eigen::Matrix<double,6,6> const& covariance)
: mean_                       (mean),
  cholesky_                   (setCholeskyMatrix(covariance)),
  max_orientation_deviation_  (std::numeric_limits<double>::max()),
  max_position_deviation_     (std::numeric_limits<double>::max()),
  generator_                  (),
  distribution_               (0.0, 1.0)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseSampler::sample(Eigen::Affine3d const& T) const
{
  Eigen::Matrix<double,6,1> tau;
  for(int i=0; i<6; i++)
    tau(i) = distribution_(generator_);
  tau = cholesky_ * tau;
  for(int i=0; i<3; i++) // <- Bound the orientation error
    tau(i) = std::max(-max_orientation_deviation_, std::min(tau(i), max_orientation_deviation_));
  for(int i=3; i<6; i++) // <- Bound the position error
    tau(i) = std::max(-max_position_deviation_, std::min(tau(i), max_position_deviation_));
  return common::so3r3::product(tau,T);
}

//--------------------------------------------------------------------------------------------------

void PoseSampler::setMaxOrientationDeviation(double max_orientation_dev)
{
  CHECK(max_orientation_dev > 0.);
  max_orientation_deviation_ = max_orientation_dev;
}

//--------------------------------------------------------------------------------------------------

void PoseSampler::setMaxPositionDeviation(double max_position_dev)
{
  CHECK(max_position_dev > 0.);
  max_position_deviation_ = max_position_dev;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> PoseSampler::setCholeskyMatrix(
  double sigma_orientation, double sigma_position)
{
  Eigen::Matrix<double,6,6> cholesky = Eigen::Matrix<double,6,6>::Identity();
  cholesky.block<3,3>(0,0) *= sigma_orientation;
  cholesky.block<3,3>(3,3) *= sigma_position;
  return cholesky;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> PoseSampler::setCholeskyMatrix(
  Eigen::Matrix<double,6,6> const& covariance)
{
  return Eigen::Matrix<double,6,6>(covariance.llt().matrixL());
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseSampler::sample(Eigen::Affine3d const& T, double sigma_r, double sigma_t,
  double maxdevr, double maxdevt)
{
  // Initialize static Gaussian samplers
  static std::default_random_engine generator;
  static std::normal_distribution<double> N(0.0,1.0);
  CHECK( maxdevr > 0 );
  CHECK( maxdevt > 0 );

  // Sample the orientation and translation increment
  Eigen::Matrix<double,6,1> tau;
  for(int i=0; i<3; i++)
    tau(i) = std::max( -maxdevr, std::min( sigma_r * N(generator), maxdevr ) );
  for(int i=3; i<6; i++)
    tau(i) = std::max( -maxdevt, std::min( sigma_t * N(generator), maxdevt ) );
  return common::so3r3::product(tau, T);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseSampler::sample(double sigma_r, double sigma_t, double maxdevr, double maxdevt)
{
  return sample(Eigen::Affine3d::Identity(), sigma_r, sigma_t, maxdevr, maxdevt);
}

//--------------------------------------------------------------------------------------------------

} // namespace common
