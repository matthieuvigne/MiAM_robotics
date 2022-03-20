#ifndef VISION_BOARD_HPP
#define VISION_BOARD_HPP

#include <opencv2/aruco.hpp>

#include <network/server.hpp>
#include <vision/camera_thread.hpp>

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

private:

  struct Board {
    double height;
    double width;
  }; // struct Board

private:

  Board board_;
  CameraThread::UniquePtr camera_thread_ptr_;
  //~ network::Server::UniquePtr server_ptr_;
  bool turn_off_ = false;

}; // class Module

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_BOARD_HPP
