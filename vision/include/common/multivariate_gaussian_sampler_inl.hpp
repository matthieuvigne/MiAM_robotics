#ifndef COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP
#define COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

template<int N>
MultivariateGaussianSampler<N>::MultivariateGaussianSampler(
  Eigen::Matrix<double,N,1> const& mean,
  Eigen::MatrixXd const& covariance,
  CovarianceType covariance_type)
: mean_             (mean),
  chol_             (this->initializeCholeskyMatrix(covariance, covariance_type_)),
  sigmas_           (this->initializeSigmas(covariance, covariance_type)),
  generator_        (),
  gaussian_         (0.0,1.0),
  covariance_type_  (covariance_type)
{
  // Check the provided covariance type
  if( (covariance_type != CovarianceType::Dense) && (covariance_type != CovarianceType::Sigmas) )
    throw std::runtime_error("Unknown covariance types");
}

//--------------------------------------------------------------------------------------------------

template<int N>
MultivariateGaussianSampler<N>::MultivariateGaussianSampler(
  Eigen::MatrixXd const& covariance,
  CovarianceType covariance_type)
: MultivariateGaussianSampler(Eigen::Matrix<double,N,1>::Zero(), covariance, covariance_type)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

template<int N>
Eigen::Matrix<double,N,1> MultivariateGaussianSampler<N>::sample() const
{
  // Sample from the centered normal distribution
  Eigen::Matrix<double,N,1> std_normal_sampled_vector;
  for(int i=0; i<N; i++)
    std_normal_sampled_vector(i) = this->gaussian_(this->generator_);

  // Compute the resulting vector
  Eigen::Matrix<double,N,1> result = this->mean_;
  switch(this->covariance_type_)
  {
    case CovarianceType::Dense:
      result += this->chol_*std_normal_sampled_vector;
      break;
    case CovarianceType::Sigmas:
      result += this->sigmas_.cwiseProduct(std_normal_sampled_vector);
      break;
    default:
      throw std::runtime_error("Unknown covariance type");
  }
  return result;
}

//--------------------------------------------------------------------------------------------------

template<int N>
Eigen::Matrix<double,N,N> MultivariateGaussianSampler<N>::initializeCholeskyMatrix(
  Eigen::MatrixXd covariance,
  CovarianceType covariance_type)
{
  if(covariance_type == CovarianceType::Dense)
  {
    CHECK(covariance.cols() == N);
    return Eigen::Matrix<double,N,N>(covariance.llt().matrixL());
  }
  else
    return Eigen::Matrix<double,N,N>::Zero();
}

//--------------------------------------------------------------------------------------------------

template<int N>
Eigen::Matrix<double,N,1> MultivariateGaussianSampler<N>::initializeSigmas(
  Eigen::MatrixXd covariance,
  CovarianceType covariance_type)
{
  if(covariance_type == CovarianceType::Sigmas)
  {
    CHECK(covariance.rows() == N);
    return Eigen::Matrix<double,N,1>(covariance);
  }
  else
    return Eigen::Matrix<double,N,1>::Zeros();
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_MULTIVARIATE_GAUSSIAN_SAMPLER_INL_HPP
