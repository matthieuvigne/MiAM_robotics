#include <memory>

#include <yaml-cpp/yaml.h>

#include <network/socket_exception.hpp>
#include <vision/module.hpp>
#include <vision/distortion_null.hpp>
#include <vision/distortion_radtan.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Module::Module(std::string const& filename)
{
  // Load the configuration file
  YAML::Node const params = YAML::LoadFile(filename);
  assert(params["board"]);
  assert(params["camera"]);
  
  // Get the board dimensions
  YAML::Node const& board = params["board"];
  assert(board.IsMap());
  this->board_.width = board["width"].as<double>();
  this->board_.height = board["height"].as<double>();

  // Get the cameras and launch the associated threads
  YAML::Node const camera_node = params["camera"];
  std::string const camera_name = "camera";
  this->camera_ptr_ = Camera::buildCameraFromYaml(camera_name, camera_node);
  this->camera_ptr_->launchThread();
  std::cout << this->camera_ptr_->print() << std::endl;
  
  // Launch the server
  int const port = 30000;
  try {
      this->server_ptr_.reset(new network::Server(port));
  } catch(network::SocketException const& e) {
      std::cout << e.description() << std::endl;
  }
  this->server_ptr_->launchThread();
}

//--------------------------------------------------------------------------------------------------

Module::~Module()
{
  // Abort the camera thread
  this->camera_ptr_->abortThread();
  this->server_ptr_->abortThread();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Camera const& Module::getCamera() const
{
  return *(this->camera_ptr_);
}

//--------------------------------------------------------------------------------------------------

std::string Module::print() const
{
  std::stringstream out;
  out << "Module:" << std::endl;
  out << "- Board width: " << this->board_.width << "m" << std::endl;
  out << "- Board height: " << this->board_.height << "m" << std::endl;
  out << this->camera_ptr_->print() << std::endl;
  return out.str();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
