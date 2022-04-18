#ifndef COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP
#define COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP

#include <random>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace common {

template<int N>
class MultivariateGaussianSampler {

public:
  POINTER_TYPEDEF(MultivariateGaussianSampler);
  DISALLOW_EVIL_CONSTRUCTORS(MultivariateGaussianSampler);
  MultivariateGaussianSampler(
    Eigen::Matrix<double,N,1> const& mean,
    Eigen::Matrix<double,N,N> const& covariance);

public:
  Eigen::Matrix<double,N,1> sample() const;

private:

  Eigen::Matrix<double,N,1> const mean_;
  Eigen::Matrix<double,N,N> const chol_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable gaussian_;

}; // class MultivariateGaussianSampler

typedef MultivariateGaussianSampler<2> MultivariateGaussianSampler2d;
typedef MultivariateGaussianSampler<3> MultivariateGaussianSampler3d;

} // namespace common

#include <common/multivariate_gaussian_sampler_inl.hpp>

#endif // COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP
