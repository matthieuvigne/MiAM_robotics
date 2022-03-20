#ifndef VISION_BOARD_HPP
#define VISION_BOARD_HPP

#include <opencv2/aruco.hpp>

#include <network/server.hpp>
#include <vision/camera.hpp>

/* Vision module
 * -------------
 * TODO : during initialization : should spot the cameras + robot wifi network
 * */

namespace vision {

//--------------------------------------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------------------------------------

// TODO

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Module {

public:

  POINTER_TYPEDEF(Module);

public:

  DISALLOW_EVIL_CONSTRUCTORS(Module);
  Module(std::string const& filename);
  virtual ~Module();

public:

  // Camera information
  Camera const& getCamera(size_t camera_idx) const;
  inline size_t getNumCameras() const;

  // Print
  std::string print() const;

private:

  struct Board {
    double height;
    double width;
  }; // struct Board

private:

  // Board and cameras
  Board board_;
  std::vector<Camera::UniquePtr> cameras_;
  bool turn_off_ = false;

  // Server socket
  network::Server::UniquePtr server_ptr_;

}; // class Module

//--------------------------------------------------------------------------------------------------
// INLINE FUNCTIONS
//--------------------------------------------------------------------------------------------------

size_t Module::getNumCameras() const
{
  return this->cameras_.size();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_BOARD_HPP
