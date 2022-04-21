#include <map>
#include <random>

#include <camera/camera_pose_filter.hpp>
#include <common/gaussian_sampler.hpp>
#include <common/maths.hpp>
#include <common/pose_sampler.hpp>

int main(int argc, char* argv[])
{
  //------------------------------------------------------------------------------------------------
  // Set the true geometry of the scene
  //------------------------------------------------------------------------------------------------
  
  // Standard deviations for orientation and translation
  double constexpr sigma_r = 5.0*M_PI/180.;
  double constexpr sigma_t = 2e-2;
  
  // Sample the initial pose of the camera
  Eigen::Affine3d const TWC_est = 
      Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  Eigen::Affine3d const TWC_true = common::PoseSampler::sample(TWC_est, sigma_r, sigma_t);

  // Sample the pose of the central marker
  Eigen::Affine3d const TWM_est =
      Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d const TWM_true = common::PoseSampler::sample(TWM_est, sigma_r, sigma_t);
  
  // Sample the transformation between the camera and the rotor's axis
  Eigen::Affine3d const TRC_est =
      Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  Eigen::Affine3d const TRC_true = common::PoseSampler::sample(TRC_est, sigma_r, sigma_t);

  //------------------------------------------------------------------------------------------------
  // Initialize the camera pose filter
  //------------------------------------------------------------------------------------------------

  // Initialize the state and the covariance
  camera::CameraPoseFilter filter(TWM_est, TRC_est, sigma_r, sigma_t);
  Eigen::Matrix<double,6,6> cov_TWC = Eigen::Matrix<double,6,6>::Identity();
  cov_TWC.block<3,3>(0,0) *= std::pow(sigma_r, 2.0);
  cov_TWC.block<3,3>(3,3) *= std::pow(sigma_t, 2.0);
  filter.setStateAndCovariance(camera::CameraPoseFilter::InitType::T_WC, TWC_est, cov_TWC);

  //------------------------------------------------------------------------------------------------
  // Simulation of the process
  //------------------------------------------------------------------------------------------------
  
  // Simulate the process and the measurements
  int constexpr max_idx = 20;
  double const max_angle_rad = common::convertDegreeToRadian(30.0);
  double const angle_step_rad = common::convertDegreeToRadian(5.0);
  for(int idx=0; idx<max_idx; idx++)
  {
    // Move the camera and predict
    // [TODO]
    
    // Update with measurements
    // [TODO]
  }
  
  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
