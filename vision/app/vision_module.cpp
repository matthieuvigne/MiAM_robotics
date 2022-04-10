#include <iostream>

#include <module/module.hpp>

// Camera thread: at each turn, increment the camera, except if reception of a specific request
// Server thread: receive and process the client's requests

// Main routine
int main(int argc, char* argv[])
{
  // Set the camera parameters
  camera::CameraParams camera_params;
  camera_params.name = "camera";
  camera_params.resolution[camera::CameraParams::WIDTH]  = 1280;
  camera_params.resolution[camera::CameraParams::HEIGHT] = 960;
  camera_params.intrinsics[camera::CameraParams::FX] = 1368.818052261177;
  camera_params.intrinsics[camera::CameraParams::FY] = 1358.929438179962;
  camera_params.intrinsics[camera::CameraParams::CX] = 542.3083210278664;
  camera_params.intrinsics[camera::CameraParams::CY] = 476.3511642509686;
  camera_params.distortion_model = camera::DistortionModel::Type::NoDistortion;
  camera_params.distortion_coeffs = {
     0.3321859739578252, 
    -1.130115755363346,
     0.001940341517564527,
    -0.03055278893902099,
     1.429087154492101};
  camera_params.pose = Eigen::Affine3d::Identity();

  // Set the module parameters
  module::ModuleParams parameters;
  parameters.board_height = 2.0;
  parameters.board_width  = 3.0;
  parameters.camera_params = camera_params;
  parameters.T_WM = Eigen::Affine3d::Identity();
  parameters.T_RC = Eigen::Affine3d::Identity();
  parameters.cov_T_RC = Eigen::Matrix<double,6,6>::Identity();

  // Initialize the vision module
  // -> Launches the internal camera and server threads
  module::Module::UniquePtr module_ptr(new module::Module(parameters));
  module_ptr->join();
  return EXIT_SUCCESS;
}
