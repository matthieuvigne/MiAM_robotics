#include <common/gaussian_sampler.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

GaussianSampler::GaussianSampler()
: GaussianSampler(0.0, 1.0)
{}

//--------------------------------------------------------------------------------------------------

GaussianSampler::GaussianSampler(double mean, double sigma, double max_deviation)
: max_deviation_   (max_deviation),
  distribution_    (mean, sigma)
{
  CHECK(sigma >= 0.);
  CHECK(max_deviation >= 0.);
}

//--------------------------------------------------------------------------------------------------

} // namespace common
