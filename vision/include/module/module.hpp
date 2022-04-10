#ifndef VISION_BOARD_HPP
#define VISION_BOARD_HPP

#include <opencv2/aruco.hpp>

#include <network/server_thread.hpp>
#include <camera/camera_thread.hpp>

/* Vision module
 * -------------
 * TODO : during initialization : should spot the cameras + robot wifi network
 * */

namespace module {

//--------------------------------------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------------------------------------

struct ModuleParams {

  // Board and central marker
  int board_height;
  int board_width;
  Eigen::Affine3d T_WM;
  
  // Camera
  camera::CameraParams camera_params;
  Eigen::Affine3d T_RC;
  Eigen::Matrix<double,6,6> cov_T_RC;
  
}; // struct ModuleParams

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Module {

public:

  POINTER_TYPEDEF(Module);
  DISALLOW_EVIL_CONSTRUCTORS(Module);
  Module(std::string const& filename);
  Module(ModuleParams const& params);
  virtual ~Module();
  
public:

  void join();

private:

  struct Board {
    double height;
    double width;
  }; // struct Board

private:

  Board board_;
  camera::CameraThread::UniquePtr camera_thread_ptr_;
  network::ServerThread::UniquePtr server_thread_ptr_;
  bool turn_off_ = false;

}; // class Module

//--------------------------------------------------------------------------------------------------

} // namespace module

#endif // VISION_BOARD_HPP
