#include <iostream>

#include <common/gaussian_sampler.hpp>
#include <common/maths.hpp>
#include <common/pose_gaussian_sampler.hpp>

//--------------------------------------------------------------------------------------------------
// Utility functions
//--------------------------------------------------------------------------------------------------

double mean(std::vector<double> const& values)
{
  double sum = std::accumulate(values.cbegin(), values.cend(), 0.0);
  return sum / values.size();
}

//--------------------------------------------------------------------------------------------------

double stddev(std::vector<double> const& values)
{
  double mean = std::accumulate(values.cbegin(), values.cend(), 0.0) / values.size();
  double sqsum = std::inner_product(values.cbegin(), values.cend(), values.cbegin(), 0.0);
  return std::sqrt( sqsum / values.size() - mean * mean );
}

//--------------------------------------------------------------------------------------------------
// Main test routine
//--------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  /* ------------------------------------------------------- *
   * Checking Jacobians                                      *
   * ------------------------------------------------------- *
   * f(tau + T) = Exp( J*tau ) * f(T)                        *
   * => J*tau = Log( f(tau + T) * f(T)^(-1) )                *
   * Beware, Exp, Log et * are defined over SO3*R3, not SE3  *
   * ------------------------------------------------------- *
   */

  // Initialize the pose sampler
  double constexpr sigma_orientation = M_PI;
  double constexpr sigma_position = 10.0;
  common::PoseGaussianSampler sampler(Eigen::Affine3d::Identity(), sigma_orientation, sigma_position);

  // Initialize the Gaussian sampler
  common::GaussianSampler orientation_sampler(0,3*M_PI/180.);
  orientation_sampler.setMaxDeviation(3*M_PI/180.);
  common::GaussianSampler position_sampler(0,1e-2);
  position_sampler.setMaxDeviation(1e-2);

  // Check the identity
  // ------------------
  int constexpr num_tests = 1e3;
  std::vector<double> residuals(num_tests);
  for(int test_idx=0; test_idx<num_tests; test_idx++)
  {
    // Sample
    Eigen::Affine3d T = sampler.sample();
    Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Ones();
    for(int i=0; i<3; i++) tau(i) = orientation_sampler.sample();
    for(int i=3; i<6; i++) tau(i) = position_sampler.sample();
    Eigen::Affine3d tT = common::so3r3::product(tau, T);
    Eigen::Affine3d invT = common::so3r3::inverse(T);
    Eigen::Matrix<double,6,1> residual =
      common::so3r3::logMap( common::so3r3::product(tT, invT) ) - tau;
    residuals[test_idx] = residual.norm();
  }
  std::cout << "Identity -> mean = " << mean(residuals) << std::endl;
  std::cout << "Identity -> stddev = " << stddev(residuals) << std::endl;
  std::cout << "Identity -> max = " << *std::max_element(residuals.cbegin(), residuals.cend())
    << std::endl << std::endl;
  
  // Check the left SE3 pose product Jacobian matrix
  // -----------------------------------------------
  for(int test_idx=0; test_idx<num_tests; test_idx++)
  {
    // Sample
    Eigen::Affine3d T1 = sampler.sample();
    Eigen::Affine3d T2 = sampler.sample();
    Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Ones();
    for(int i=0; i<3; i++) tau(i) = orientation_sampler.sample();
    for(int i=3; i<6; i++) tau(i) = position_sampler.sample();
    
    // Compute the residual
    Eigen::Matrix<double,6,6> J = common::so3r3::leftSe3ProductJacobian(T1,T2);
    Eigen::Affine3d tT1T2 = common::so3r3::product(tau,T1) * T2;
    Eigen::Affine3d invT1T2 = common::so3r3::inverse( T1*T2 );
    Eigen::Matrix<double,6,1> residual = 
      common::so3r3::logMap( common::so3r3::product( tT1T2, invT1T2 ) ) - J*tau;
    residuals[test_idx] = residual.norm();
  }
  std::cout << "Left Jacobian -> mean = " << mean(residuals) << std::endl;
  std::cout << "Left Jacobian -> stddev = " << stddev(residuals) << std::endl;
  std::cout << "Left Jacobian -> max = " << *std::max_element(residuals.cbegin(), residuals.cend())
    << std::endl << std::endl;

  // Check the right pose product Jacobian matrix
  // --------------------------------------------
  for(int test_idx=0; test_idx<num_tests; test_idx++)
  {
    // Sample
    Eigen::Affine3d T1 = sampler.sample();
    Eigen::Affine3d T2 = sampler.sample();
    Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Ones();
    for(int i=0; i<3; i++) tau(i) = orientation_sampler.sample();
    for(int i=3; i<6; i++) tau(i) = position_sampler.sample();

    // Compute the residual
    Eigen::Matrix<double,6,6> J = common::so3r3::rightSe3ProductJacobian(T1, T2);
    Eigen::Affine3d T1tT2 = T1 * common::so3r3::product(tau,T2);
    Eigen::Affine3d invT1T2 = common::so3r3::inverse( T1*T2 );
    Eigen::Matrix<double,6,1> residual = 
      common::so3r3::logMap( common::so3r3::product(T1tT2, invT1T2) ) - J*tau;
    residuals[test_idx] = residual.norm();
  }
  std::cout << "Right Jacobian -> mean = " << mean(residuals) << std::endl;
  std::cout << "Right Jacobian -> stddev = " << stddev(residuals) << std::endl;
  std::cout << "Right Jacobian -> max = " << *std::max_element(residuals.cbegin(), residuals.cend())
    << std::endl << std::endl;

  // Check the pose inverse Jacobian matrix
  // --------------------------------------
  for(int test_idx=0; test_idx<num_tests; test_idx++)
  {
    // Sample
    Eigen::Affine3d T1 = sampler.sample();
    Eigen::Affine3d T2 = sampler.sample();
    Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Ones();
    for(int i=0; i<3; i++) tau(i) = orientation_sampler.sample();
    for(int i=3; i<6; i++) tau(i) = position_sampler.sample();

    // Compute the residual
    Eigen::Matrix<double,6,6> J = common::so3r3::se3InverseJacobian(T1);
    Eigen::Affine3d invtT1 = common::so3r3::product(tau,T1).inverse();
    Eigen::Affine3d invT1 = common::so3r3::inverse( T1.inverse() );
    Eigen::Matrix<double,6,1> residual =
      common::so3r3::logMap( common::so3r3::product(invtT1, invT1) ) - J*tau;
    residuals[test_idx] = residual.norm();
  }
  std::cout << "Pose inverse -> mean = " << mean(residuals) << std::endl;
  std::cout << "Pose inverse -> stddev = " << stddev(residuals) << std::endl;
  std::cout << "Pose inverse -> max = " << *std::max_element(residuals.cbegin(), residuals.cend())
    << std::endl;

  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
