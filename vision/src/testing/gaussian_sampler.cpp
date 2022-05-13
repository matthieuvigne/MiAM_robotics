#include <testing/gaussian_sampler.hpp>

namespace testing {

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
  CHECK(sigma > 0.);
  CHECK(max_deviation > 0.);
}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

double GaussianSampler::sample(double mean, double stddev, double maxdev)
{
  CHECK( maxdev > 0);
  static std::default_random_engine generator;
  static std::normal_distribution<double> N(0.0,1.0);
  return std::max( - maxdev, std::min( mean + stddev * N(generator), maxdev ) );
}

//--------------------------------------------------------------------------------------------------

} // namespace testing
