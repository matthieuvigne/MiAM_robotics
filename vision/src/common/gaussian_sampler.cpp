#include <common/gaussian_sampler.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

GaussianSampler::GaussianSampler()
: GaussianSampler(0.0, 1.0)
{}

//--------------------------------------------------------------------------------------------------

GaussianSampler::GaussianSampler(double mean, double sigma, double max_dispersion)
: max_dispersion_   (max_dispersion),
  distribution_     (mean, sigma)
{}

//--------------------------------------------------------------------------------------------------

} // namespace common
