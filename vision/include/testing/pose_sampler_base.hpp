#ifndef TESTING_POSE_SAMPLER_BASE_HPP
#define TESTING_POSE_SAMPLER_BASE_HPP

#include <random>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class PoseSamplerBase {

public:
  enum class Type { UNIFORM, GAUSSIAN };

public:
  POINTER_TYPEDEF(PoseSamplerBase);
  DISALLOW_EVIL_CONSTRUCTORS(PoseSamplerBase);
  PoseSamplerBase(Type type);

public:

  GET_AS_CASTER;
  virtual Eigen::Affine3d sample() const = 0;
  inline Type getType() const { return type_; }

protected:

  Type const type_;
  std::default_random_engine mutable generator_;

}; // class PoseSamplerBase

//--------------------------------------------------------------------------------------------------

} // namespace testing

#endif // TESTING_POSE_SAMPLER_BASE_HPP
