#include <memory>

#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include <yaml-cpp/yaml.h>

#include <common/yaml_serialization.hpp>
#include <module/module.hpp>
#include <network/socket_exception.hpp>
#include <vision/distortion_null.hpp>
#include <vision/distortion_radtan.hpp>

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
  vision::Camera::UniquePtr camera_ptr =
    vision::Camera::buildCameraFromYaml(camera_name, camera_node);
  Eigen::Affine3d const T_WM =
    common::yaml_serialization::deserializePose(params["T_WM"]);
  Eigen::Affine3d const T_RC =
    common::yaml_serialization::deserializePose(params["T_RC"]);
  Eigen::Matrix<double,6,6> const cov_T_RC =
    common::yaml_serialization::deserializePoseCovariance(params["cov_T_RC"]);
  this->camera_thread_ptr_.reset(new vision::CameraThread(T_WM, T_RC, cov_T_RC,
    std::move(camera_ptr)));

  // Launch the server's thread
  //~ int const port = 30000;
  //~ try {
      //~ this->server_thread_ptr_.reset(new network::ServerThread(port));
  //~ } catch(network::SocketException const& e) {
      //~ std::cout << e.description() << std::endl;
  //~ }
}

//--------------------------------------------------------------------------------------------------

Module::Module(ModuleParams const& params)
{
  // Setup RPi GPIO for servo control.
  RPi_enableGPIO();
  RPi_enablePWM(true, false);
  RPi_setPWMClock(PiPWMClockFrequency::F1200kHz);

  // Get the board dimensions
  this->board_.height = params.board_height;
  this->board_.width = params.board_width;
  
  // Build the camera and launch its thread
  vision::Camera::UniquePtr camera_ptr(new vision::Camera(params.camera_params));
  Eigen::Affine3d const& T_WM = params.T_WM;
  Eigen::Affine3d const& T_RC = params.T_RC;
  Eigen::Matrix<double,6,6> const& cov_T_RC = params.cov_T_RC;
  this->camera_thread_ptr_.reset(new vision::CameraThread(T_WM, T_RC, cov_T_RC,
    std::move(camera_ptr)));
    
  // Launch the server's thread
  //~ int const port = 30000;
  //~ try {
    //~ this->server_thread_ptr_.reset(new network::ServerThread(port));
  //~ } catch(network::SocketException const& e) {
    //~ std::cout << e.description() << std::endl;
  //~ }
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

} // namespace module
