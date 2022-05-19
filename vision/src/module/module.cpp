#include <memory>

#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include <yaml-cpp/yaml.h>

#include <camera/distortion_null.hpp>
#include <camera/distortion_radtan.hpp>
#include <common/logger.hpp>
#include <common/maths.hpp>
#include <module/module.hpp>
#include <network/socket_exception.hpp>

namespace module {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Module::Module()
{
  // Setup RPi GPIO for servo control.
  #ifdef RPI4
    #if !USE_TEST_BENCH
      system("echo 0 > /sys/class/pwm/pwmchip0/export");
      usleep(50000);
      system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable");
      usleep(50000);
      system("echo 20000000 > /sys/class/pwm/pwmchip0/pwm0/period");
      usleep(50000);
      system("echo 1500000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
      usleep(50000);
      system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");
    #endif
  #endif

  // Build the camera and launch its thread
  camera_thread_ptr_.reset(new camera::CameraThread);

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

} // namespace module
