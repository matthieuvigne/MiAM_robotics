#include <algorithm>

#include <common/gaussian_sampler.hpp>

int main(int argc, char* argv[])
{
  // Initialize the Gaussian sampler
  common::GaussianSampler sampler(0.0, 1.0);
  sampler.setMaxDeviation(3.0);
  
  // Sample from the unit Gaussian law
  size_t max_samples = 1e4;
  std::vector<double> values(max_samples);
  for(size_t i=0u; i<max_samples; i++)
    values[i] = sampler.sample();

  // Check the statistics are coherent
  double const sum = std::accumulate(values.cbegin(), values.cend(), 0.0);
  double const mean = sum / values.size();
  double const square_sum = std::inner_product(values.cbegin(), values.cend(), values.cbegin(), 0.0);
  double const stddev = std::sqrt(square_sum / values.size() - mean*mean);
  double const min_value = *std::min_element(values.cbegin(), values.cend());
  double const max_value = *std::max_element(values.cbegin(), values.cend());

  // Print the results
  std::cout << "Mean: " << mean << std::endl;
  std::cout << "Deviation: " << stddev << std::endl;
  std::cout << "Min value: " << min_value << std::endl;
  std::cout << "Max value: " << max_value << std::endl;
  
  return EXIT_SUCCESS;
}
