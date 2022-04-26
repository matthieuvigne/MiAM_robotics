#include <common/maths.hpp>
#include <common/pose_uniform_sampler.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

PoseUniformSampler::PoseUniformSampler(
  double min_wx, double min_wy, double min_wz,
  double max_wx, double max_wy, double max_wz,
  double min_tx, double min_ty, double min_tz,
  double max_tx, double max_ty, double max_tz)
: PoseUniformSampler(
    Eigen::Vector3d(min_wx, min_wy, min_wz),
    Eigen::Vector3d(max_wx, max_wy, max_wz),
    Eigen::Vector3d(min_tx, min_ty, min_tz),
    Eigen::Vector3d(max_tx, max_ty, max_tz))
{}

//--------------------------------------------------------------------------------------------------

PoseUniformSampler::PoseUniformSampler(
  Eigen::Vector3d const& min_w, Eigen::Vector3d const& max_w,
  Eigen::Vector3d const& min_t, Eigen::Vector3d const& max_t)
: PoseSamplerBase   (Type::UNIFORM),
  U_ (0.0,1.0),
  min_w_ (min_w), max_w_ (max_w), 
  min_t_ (min_t), max_t_ (max_t)
{
  CHECK(((max_w_ - min_w_).array() >= 0).all());
  CHECK(((max_t_ - min_t_).array() >= 0).all());
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d PoseUniformSampler::sample() const
{
  Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Ones();
  for(int i=0; i<3; i++)
  {
    double const coeff = U_(generator_);
    CHECK( (coeff>=0.) && (coeff<=1.0) );
    tau(i) = min_w_(i) + coeff * (max_w_(i) - min_w_(i));
    tau(i) = std::fmod(tau(i),2*M_PI);
  }
  for(int i=0; i<3; i++)
  {
    double const coeff = U_(generator_);
    CHECK( (coeff>=0.) && (coeff<=1.0) );
    tau(3+i) = min_t_(i) + coeff * (max_t_(i) - min_t_(i));
  }
  return common::so3r3::expMap(tau);
}

//--------------------------------------------------------------------------------------------------

} // namespace common
