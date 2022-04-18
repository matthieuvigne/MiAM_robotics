#ifndef COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP
#define COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

template<int N>
MultivariateGaussianSampler<N>::MultivariateGaussianSampler(
  Eigen::Matrix<double,N,1> const& mean,
  Eigen::Matrix<double,N,N> const& covariance)
: mean_       (mean),
  chol_       (covariance.llt().matrixL()),
  generator_  (),
  gaussian_   (0.0,1.0)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

template<int N>
Eigen::Matrix<double,N,1> MultivariateGaussianSampler<N>::sample() const
{
  Eigen::Matrix<double,N,1> std_normal_sampled_vector;
  for(int i=0; i<N; i++)
    std_normal_sampled_vector(i) = this->gaussian_(this->generator_);
  return (this->mean_ + this->chol_*std_normal_sampled_vector);
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP
