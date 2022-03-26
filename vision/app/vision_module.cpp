#include <iostream>

#include <module/module.hpp>

// Camera thread: at each turn, increment the camera, except if reception of a specific request
// Server thread: receive and process the client's requests

// Main routine
int main(int argc, char* argv[])
{
  // Set the camera parameters
  vision::CameraParams camera_params;
  camera_params.name = "camera";
  camera_params.resolution[vision::CameraParams::WIDTH]  = 752;
  camera_params.resolution[vision::CameraParams::HEIGHT] = 480;
  camera_params.intrinsics[vision::CameraParams::FX] = 450;
  camera_params.intrinsics[vision::CameraParams::FY] = 450;
  camera_params.intrinsics[vision::CameraParams::CX] = 376;
  camera_params.intrinsics[vision::CameraParams::CY] = 240;
  camera_params.distortion_model = vision::DistortionModel::Type::RadTan;
  camera_params.distortion_coeffs = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0};
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
}
