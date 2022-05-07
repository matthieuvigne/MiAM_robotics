#include <memory>

#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include <yaml-cpp/yaml.h>

#include <camera/distortion_null.hpp>
#include <camera/distortion_radtan.hpp>
#include <common/logger.hpp>
#include <common/maths.hpp>
#include <common/yaml_serialization.hpp>
#include <module/module.hpp>
#include <network/socket_exception.hpp>

namespace module {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Module::Module(ModuleParams const& params)
{
  // Setup RPi GPIO for servo control.
  #ifdef RPI4
  RPi_enableGPIO();
  RPi_enablePWM(true, false);
  RPi_setPWMClock(PiPWMClockFrequency::F1200kHz);
  #endif

  // Get the board dimensions
  board_.height = params.board_height;
  board_.width = params.board_width;
  
  // Build the camera and launch its thread
  camera::CameraThread::Params thread_params;
  thread_params.camera_ptr.reset(new camera::Camera(params.camera_params));
  thread_params.T_WM = params.T_WM;
  thread_params.T_RC = params.T_RC;
  thread_params.cov_T_RC = params.cov_T_RC;
  camera_thread_ptr_.reset(new camera::CameraThread(thread_params));

  // Launch the server's thread
  int const port = 30000;
  try {
    server_thread_ptr_.reset(new network::ServerThread(port));
    server_thread_ptr_->setCameraThread(camera_thread_ptr_.get());
  } catch(network::SocketException const& e) {
    CONSOLE << e.description();
  }
}

//--------------------------------------------------------------------------------------------------

Module::~Module(){}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Module::join()
{
  camera_thread_ptr_->join();
  server_thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------

ModuleParams ModuleParams::getDefaultParams()
{
  module::ModuleParams parameters;
  parameters.board_height = 2.0;
  parameters.board_width  = 3.0;
  parameters.camera_params = camera::Camera::Params::getDefaultParams();
  parameters.T_WM =
      Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd();
  parameters.T_RC =
      Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  parameters.cov_T_RC = Eigen::Matrix<double,6,6>::Identity();
  for(int i=0; i<3; i++) parameters.cov_T_RC(i,i) = std::pow(1.0*RAD,2.0);
  for(int i=3; i<6; i++) parameters.cov_T_RC(i,i) = std::pow(5e-3,2.0);
  return parameters;
}

//--------------------------------------------------------------------------------------------------

} // namespace module
