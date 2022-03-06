#include <memory>

#include <yaml-cpp/yaml.h>

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
  assert(params["cameras"]);
  assert(params["beacons"]);
  
  // Get the board dimensions
  YAML::Node const& board = params["board"];
  assert(board.IsMap());
  this->board_.width = board["width"].as<double>();
  this->board_.height = board["height"].as<double>();

  // Get the beacons
  //~ YAML::Node const beacons = params["beacons"];
  //~ assert(beacons.IsMap());
  //~ for(YAML::const_iterator it = beacons.begin(); it != beacons.end(); ++it)
  //~ {
    // Build the beacon
  //~ }

  // Get the cameras and launch the associated threads
  YAML::Node const cameras = params["cameras"];
  assert(cameras.IsMap());
  for(YAML::const_iterator it = cameras.begin(); it != cameras.end(); ++it)
  {
    std::string const camera_name = it->first.as<std::string>();
    YAML::Node const& camera_node = it->second;
    Camera::UniquePtr camera_ptr = Camera::buildCameraFromYaml(camera_name, camera_node);
    camera_ptr->launchThread();
    std::cout << camera_ptr->print() << std::endl;
    this->cameras_.push_back(std::move(camera_ptr));
  }
}

//--------------------------------------------------------------------------------------------------

Module::~Module()
{
  // Abort all camera threads
  size_t const num_cameras = this->cameras_.size();
  for(size_t camera_idx=0u; camera_idx<num_cameras; camera_idx++)
    this->cameras_[camera_idx]->abortThread();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Camera const& Module::getCamera(size_t camera_idx) const
{
  assert(camera_idx < this->cameras_.size());
  return *(this->cameras_[camera_idx]);
}

//--------------------------------------------------------------------------------------------------

std::string Module::print() const
{
  std::stringstream out;
  out << "Module:" << std::endl;
  out << "- Board width: " << this->board_.width << "m" << std::endl;
  out << "- Board height: " << this->board_.height << "m" << std::endl;
  size_t const num_cameras = this->cameras_.size();
  for(size_t camera_idx=0u; camera_idx<num_cameras; ++camera_idx)
  {
    out << "Camera " << camera_idx << ":" << std::endl;
    out << this->cameras_[camera_idx]->print() << std::endl;
  }
  return out.str();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
