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
  camera_params.intrinsics[camera::CameraParams::FX] = 7.0424945444991499e+02;
  camera_params.intrinsics[camera::CameraParams::FY] = 6.9896472232651024e+02;
  camera_params.intrinsics[camera::CameraParams::CX] = camera_params.resolution[camera::CameraParams::WIDTH] / 2;
  camera_params.intrinsics[camera::CameraParams::CY] = camera_params.resolution[camera::CameraParams::HEIGHT] / 2;
  camera_params.distortion_model = camera::DistortionModel::Type::RadTan;
  camera_params.distortion_coeffs = {
     2.5432953861499258e-01,
    -5.8481797342726261e-01,
    -9.7755009212097837e-04,
    -2.9062089311574019e-02,
     3.1200641998405720e-01};
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
