#include <iostream>

#include <module/module.hpp>
#ifdef USE_TEST_BENCH
  #include <module/test_bench.hpp>
#endif

// Camera thread: at each turn, increment the camera, except if reception of a specific request
// Server thread: receive and process the client's requests

// Main routine
int main(int argc, char* argv[])
{
  // Initialize the test bench if needed
  #ifdef USE_TEST_BENCH
    LOG("Initializing the test bench");
    module::TestBench::initializeTestBench();
    LOG("Test bench initialized");
  #else
    LOG("USE TEST BENCH not defined");
  #endif

  // Set the camera parameters
  camera::CameraParams camera_params;
  camera_params.name = "camera";
  camera_params.resolution[camera::CameraParams::WIDTH]  = 1280;
  camera_params.resolution[camera::CameraParams::HEIGHT] = 960;
  camera_params.intrinsics[camera::CameraParams::FX] = 1368.818;
  camera_params.intrinsics[camera::CameraParams::FY] = 1358.929;
  camera_params.intrinsics[camera::CameraParams::CX] =  542.308;
  camera_params.intrinsics[camera::CameraParams::CY] =  476.351;
  camera_params.distortion_model = camera::DistortionModel::Type::NoDistortion;
  //~ camera_params.distortion_coeffs = {0.332, -1.130, 0.002, -0.031, 1.429};
  camera_params.distortion_coeffs = {0., 0., 0., 0., 0.};
  camera_params.pose = Eigen::Affine3d::Identity();

  // Set the module parameters
  module::ModuleParams parameters;
  parameters.board_height = 2.0;
  parameters.board_width  = 3.0;
  parameters.camera_params = camera_params;
  parameters.T_WM =
      Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  parameters.T_RC =
      Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  parameters.cov_T_RC = Eigen::Matrix<double,6,6>::Identity();
  for(int i=0; i<3; i++) parameters.cov_T_RC(i,i) = std::pow(1.0*M_PI/180.,2.0);
  for(int i=3; i<6; i++) parameters.cov_T_RC(i,i) = std::pow(5e-3,2.0);

  // Initialize the vision module
  // -> Launches the internal camera and server threads
  module::Module::UniquePtr module_ptr(new module::Module(parameters));
  module_ptr->join();
  return EXIT_SUCCESS;
}
