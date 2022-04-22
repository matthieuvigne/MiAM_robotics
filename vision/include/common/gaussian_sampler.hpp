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
  inline void setMaxDeviation(double max_deviation);
  static double sample(double mean, double stddev,
    double maxdev = std::numeric_limits<double>::infinity());

private:
  double max_deviation_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable distribution_;

}; // class GaussianSampler

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

double GaussianSampler::sample() const
{
  double result = distribution_(generator_);
  double const deviation = result - distribution_.mean();
  if(deviation > 0.)
    result = distribution_.mean() + std::min(deviation,  max_deviation_);
  else
    result = distribution_.mean() + std::max(deviation, -max_deviation_);
  return result;
}

//--------------------------------------------------------------------------------------------------

void GaussianSampler::setMaxDeviation(double max_deviation)
{
  CHECK(max_deviation > 0.);
  max_deviation_ = max_deviation;
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_GAUSSIAN_SAMPLER_HPP
