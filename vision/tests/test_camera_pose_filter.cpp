#include <fstream>
#include <iostream>
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
  double const sigma_r = common::convertDegreeToRadian(5.0);
  double const sigma_t = 2e-2;
  
  // Sample the initial pose of the camera
  Eigen::Affine3d TWC_est = 
      Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  Eigen::Affine3d TWC_true = common::PoseSampler::sample(TWC_est, sigma_r, sigma_t);

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
  
  // Initialize the results file
  Eigen::IOFormat const style(3, 0, ";");
  std::ofstream results_file;
  results_file.open("results.csv");
  results_file << "# idx; wx; wy; wz; tx; ty; tz" << std::endl;
  results_file.close();
  
  // Simulate the process and the measurements
  int constexpr max_idx = 20;
  double camera_angle_rad = 0.0;
  double const max_camera_angle_rad = common::convertDegreeToRadian(30.0);
  double camera_angle_step_rad = common::convertDegreeToRadian(5.0);

  for(int idx=0; idx<max_idx; idx++)
  {
    // Move the camera and predict
    Eigen::Matrix<double,6,1> tau;
    double constexpr sigma_w = 0.5*M_PI/180.;
    if(std::fabs(camera_angle_rad) >= max_camera_angle_rad)
      camera_angle_step_rad *= -1.;
    double const dw = common::GaussianSampler::sample(camera_angle_step_rad,sigma_w);
    tau << 0., 0., dw, 0., 0., 0.;
    Eigen::Affine3d const TRkRkp1 = common::so3r3::expMap(tau);
    TWC_true = TWC_true * TRC_true.inverse() * TRkRkp1 * TRC_true;
    camera_angle_rad += dw;
    filter.predict(camera_angle_step_rad, sigma_w, camera::CameraPoseFilter::Axis::Y);
    
    // Update with measurements
    double constexpr sigma_mesr = 3.0*M_PI/180.;
    double constexpr sigma_mest = 2e-2;
    Eigen::Affine3d const TCM_true = TWC_true.inverse() * TWM_true;
    Eigen::Affine3d const TCM_measured = common::PoseSampler::sample(TCM_true, sigma_mesr, sigma_mest);
    Eigen::Matrix<double,6,6> cov_TCM = Eigen::Matrix<double,6,6>::Identity();
    cov_TCM.block<3,3>(0,0) *= std::pow(sigma_mesr,2.0);
    cov_TCM.block<3,3>(3,3) *= std::pow(sigma_mest,2.0);
    filter.update(TCM_measured, cov_TCM);
    
    // Get the current estimated state
    TWC_est = filter.getState(); 
    Eigen::Matrix<double,6,1> const error = common::so3r3::boxminus(TWC_est, TWC_true);
    results_file.open("results.csv", std::ios::app);
    results_file << error.transpose().format(style) << std::endl;
    results_file.close();
  }
  
  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
