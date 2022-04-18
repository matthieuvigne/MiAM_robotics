#ifndef COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP
#define COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP

#include <random>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace common {

template<int N>
class MultivariateGaussianSampler {

public:
  enum class CovarianceType {Dense, Sigmas};

public:
  POINTER_TYPEDEF(MultivariateGaussianSampler);
  DISALLOW_EVIL_CONSTRUCTORS(MultivariateGaussianSampler);
  MultivariateGaussianSampler(
    Eigen::Matrix<double,N,1> const& mean,
    Eigen::MatrixXd const& covariance,
    CovarianceType covariance_type);
  MultivariateGaussianSampler(
    Eigen::MatrixXd const& covariance,
    CovarianceType covariance_types);

public:
  Eigen::Matrix<double,N,1> sample() const;

private:
  Eigen::Matrix<double,N,N> initializeCholeskyMatrix(
    Eigen::MatrixXd covariance,
    CovarianceType covariance_type);
  Eigen::Matrix<double,N,1> initializeSigmas(
    Eigen::MatrixXd covariance,
    CovarianceType covariance_type);

private:
  Eigen::Matrix<double,N,1> const mean_;
  Eigen::Matrix<double,N,N> const chol_;
  Eigen::Matrix<double,N,1> const sigmas_;
  std::default_random_engine mutable generator_;
  std::normal_distribution<double> mutable gaussian_;
  CovarianceType const covariance_type_;

}; // class MultivariateGaussianSampler

typedef MultivariateGaussianSampler<2> MultivariateGaussianSampler2d;
typedef MultivariateGaussianSampler<3> MultivariateGaussianSampler3d;

} // namespace common

#include <common/multivariate_gaussian_sampler_inl.hpp>

#endif // COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_HPP
