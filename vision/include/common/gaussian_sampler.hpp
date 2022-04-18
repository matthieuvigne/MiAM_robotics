#ifndef COMMON_GAUSSIAN_SAMPLER_HPP
#define COMMON_GAUSSIAN_SAMPLER_HPP

#include <limits>
#include <random>

#include <common/macros.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class GaussianSampler {

public:
  POINTER_TYPEDEF(GaussianSampler);
  GaussianSampler();
  GaussianSampler(
    double mean, double sigma,
    double max_dispersion = std::numeric_limits<double>::max());
  virtual ~GaussianSampler(){}

public:
  inline double sample() const;

private:
  double const max_dispersion_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable distribution_;

}; // class GaussianSampler

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

double GaussianSampler::sample() const
{
  double result = this->distribution_(this->generator_);
  result = std::min(result,  this->max_dispersion_);
  result = std::max(result, -this->max_dispersion_);
  return result;
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_GAUSSIAN_SAMPLER_HPP
