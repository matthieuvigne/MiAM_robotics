#include <random>

#include <camera/camera_pose_filter.hpp>
#include <common/maths.hpp>
#include <common/multivariate_gaussian_sampler.hpp>

int main(int argc, char* argv[])
{
  //------------------------------------------------------------------------------------------------
  // Define seeds and noise parameters
  //------------------------------------------------------------------------------------------------

  std::default_random_engine generator;
  
  // Initial pose of the camera
  double const sigma_TWCi_t = 2e-2;
  double const sigma_TWCi_r = common::convertDegreeToRadian(5.0);
  std::normal_distribution<double> gauss_TWCi_t(0.0,sigma_TWCi_t);
  std::normal_distribution<double> gauss_TWCi_r(0.0,sigma_TWCi_r);
  
  // Transformation from reference camera frame to camera frame
  double const sigma_TRC_t = 1e-3;
  double const sigma_TRC_r = common::convertDegreeToRadian(3.0);
  std::normal_distribution<double> gauss_TRC_t(0.0, sigma_TRC_t);
  std::normal_distribution<double> gauss_TRC_r(0.0, sigma_TRC_r);
  
  // Process and measurement noises
  double const sigma_TRkRkp1_r = common::convertDegreeToRadian(2.0);
  double const sigma_TCM_t = 5e-2;
  double const sigma_TCM_r = common::convertDegreeToRadian(3.0);
  std::normal_distribution<double> process_noise(0.0, sigma_TRkRkp1_r);
  std::normal_distribution<double> measurement_translation_noise(0.0, sigma_TCM_t);
  std::normal_distribution<double> measurement_orientation_noise(0.0, sigma_TCM_r);

  //------------------------------------------------------------------------------------------------
  // Define the true geometry of the scene
  //------------------------------------------------------------------------------------------------

  // Set the true camera initial pose
  Eigen::Affine3d T_WCt = 
      Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  // ... to continue

  common::MultivariateGaussianSampler3d sampler(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  Eigen::Vector3d sampled_vector = sampler.sample();
  
  //~ double sampled = process_noise(generator);

  #if 0
  //------------------------------------------------------------------------------------------------
  // Camera pose filter initialization
  //------------------------------------------------------------------------------------------------

  // Create the filter
  Eigen::Affine3d const T_WM =
    Eigen::Translation3d(1.5,1.0,0.0) * Eigen::AngleAxisd();
  Eigen::Affine3d const T_RC = Eigen::Translation3d()
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());
  Eigen::Affine3d const T_WC = Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  Eigen::Matrix<double,6,6> const cov_T_RC = 
    common::convertDegreeToRadian(3.0) * Eigen::Matrix<double,6,6>::Identity();
  Eigen::Matrix<double,6,6> cov_T_WC = Eigen::Matrix<double,6,6>::Identity();

  // Initialize the state and the covariance
  double const sigma_camera_initial_position = 2e-2;
  double const sigma_camera_initial_rotation = common::convertDegreeToRadian(5.0);
  cov_T_WC.block<3,3>(0,0) *= std::pow(sigma_camera_initial_position,2);
  cov_T_WC.block<3,3>(3,3) *= std::pow(sigma_camera_initial_rotation,2);
  camera::CameraPoseFilter filter(T_WM, T_RC, cov_T_RC);
  filter.setStateAndCovariance(camera::CameraPoseFilter::InitType::T_WC, T_WC, cov_T_WC);

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
  #endif
  
  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
