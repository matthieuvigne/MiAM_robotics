#ifndef VISION_MODULE_HPP
#define VISION_MODULE_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <vision/board.hpp>
#include <vision/messages.hpp>

namespace vision {

class Module {

public:

  POINTER_TYPEDEF(Module);

public:

  DISALLOW_EVIL_CONSTRUCTORS(Module);
  Module(std::string const& parameters_filename);

public:

  // Add the associated threads for the steps
  // - Initialization
  // - Running
  // - Wifi communication

private:

  struct CameraThread {
    size_t camera_idx;
    mutable std::mutex mtx;
    std::condition_variable con;
    vision_mgs::Image::UniquePtr image;
  }; // struct CameraThread

private:

  // Thread functions
  void boardThread();   // TODO
  void cameraThread(size_t camera_idx);
  void wifiThread();    // TODO

private:

  // Board process
  Board::UniquePtr board_ptr_;
  std::thread board_thread_;
  mutable std::mutex board_mtx_;
  std::condition_variable board_con_;
  std::vector<CameraThread> camera_threads_;

  // Wifi communication process
  // Used for receiving and sending messages and request from and to the robots
  std::thread wifi_thread_;
  std::condition_variable wifi_con_;
  
}; // class Module

} // namespace vision

#endif // VISION_MODULE_HPP
