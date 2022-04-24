#ifndef COMMON_POSE_UNIFORM_SAMPLER_HPP
#define COMMON_POSE_UNIFORM_SAMPLER_HPP

#include <common/pose_sampler_base.hpp>

namespace common {

class PoseUniformSampler : public PoseSamplerBase {

public:
  POINTER_TYPEDEF(PoseUniformSampler);
  DISALLOW_EVIL_CONSTRUCTORS(PoseUniformSampler);
  PoseUniformSampler(
    double min_wx, double min_wy, double min_wz,
    double max_wx, double max_wy, double max_wz,
    double min_tx, double min_ty, double min_tz,
    double max_tx, double max_ty, double max_tz);
  PoseUniformSampler(
    Eigen::Vector3d const& min_w, Eigen::Vector3d const& min_t,
    Eigen::Vector3d const& max_w, Eigen::Vector3d const& max_t);

public:
  Eigen::Affine3d sample() const;

private:
  std::uniform_real_distribution<double> mutable U_;
  Eigen::Vector3d const min_w_;
  Eigen::Vector3d const max_w_;
  Eigen::Vector3d const min_t_;
  Eigen::Vector3d const max_t_;

}; // class PoseUniformSampler

} // namespace common

#endif // COMMON_POSE_UNIFORM_SAMPLER_HPP
