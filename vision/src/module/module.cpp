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

Module::Module(std::string const& filename)
{
  // Setup RPi GPIO for servo control.
  RPi_enableGPIO();
  RPi_enablePWM(true, false);
  RPi_setPWMClock(PiPWMClockFrequency::F1200kHz);

  // Load the configuration file
  YAML::Node const params = YAML::LoadFile(filename);
  
  // Get the board dimensions
  YAML::Node const& board = params["board"];
  assert(board.IsMap());
  this->board_.width = board["width"].as<double>();
  this->board_.height = board["height"].as<double>();
  
  // Build the camera and launch its thread
  YAML::Node const camera_node = params["camera"];
  std::string const camera_name = "camera";
  camera::Camera::UniquePtr camera_ptr =
    camera::Camera::buildCameraFromYaml(camera_name, camera_node);
  Eigen::Affine3d const T_WM =
    common::yaml_serialization::deserializePose(params["T_WM"]);
  Eigen::Affine3d const T_RC =
    common::yaml_serialization::deserializePose(params["T_RC"]);
  Eigen::Matrix<double,6,6> const cov_T_RC =
    common::yaml_serialization::deserializePoseCovariance(params["cov_T_RC"]);
  this->camera_thread_ptr_.reset(new camera::CameraThread(T_WM, T_RC, cov_T_RC,
    std::move(camera_ptr)));

  // Launch the server's thread
  int const port = 30000;
  try {
      this->server_thread_ptr_.reset(new network::ServerThread(port));
      this->server_thread_ptr_->setCameraThread(this->camera_thread_ptr_.get());
  } catch(network::SocketException const& e) {
      std::cout << e.description() << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

Module::Module(ModuleParams const& params)
{
  LOGFILE << "Initialization of the module";

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
  LOGFILE << "Build the camera and launch the camera thread";
  camera::Camera::UniquePtr camera_ptr(new camera::Camera(params.camera_params));
  Eigen::Affine3d const& T_WM = params.T_WM;
  Eigen::Affine3d const& T_RC = params.T_RC;
  Eigen::Matrix<double,6,6> const& cov_T_RC = params.cov_T_RC;
  camera_thread_ptr_.reset(new camera::CameraThread(T_WM, T_RC, cov_T_RC, std::move(camera_ptr)));

  // Launch the server's thread
  LOGFILE << "Launch the server's thread";
  int const port = 30000;
  try {
    server_thread_ptr_.reset(new network::ServerThread(port));
    server_thread_ptr_->setCameraThread(camera_thread_ptr_.get());
  } catch(network::SocketException const& e) {
    std::cout << e.description() << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

Module::~Module(){}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Module::join()
{
  this->camera_thread_ptr_->join();
  this->server_thread_ptr_->join();
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
