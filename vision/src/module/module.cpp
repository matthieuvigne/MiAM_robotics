// #include <memory>

// #include <miam_utils/raspberry_pi/RPiGPIO.h>

// #include <camera/distortion_null.hpp>
// #include <camera/distortion_radtan.hpp>
// #include <common/logger.hpp>
// #include <common/maths.hpp>
// #include <module/module.hpp>
// #include <network/socket_exception.hpp>

// namespace module {

// //--------------------------------------------------------------------------------------------------
// // Constructor and destructor
// //--------------------------------------------------------------------------------------------------

// Module::Module(std::string const& logDirectory)
// {
//   // Setup RPi GPIO for servo control.
//   #ifdef RPI4
//     #if !USE_TEST_BENCH
//       system("echo 0 > /sys/class/pwm/pwmchip0/export");
//       usleep(50000);
//       system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable");
//       usleep(50000);
//       system("echo 20000000 > /sys/class/pwm/pwmchip0/pwm0/period");
//       usleep(50000);
//       system("echo 1400000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
//       usleep(50000);
//       system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");
//     #endif
//   #endif

//   // Build the camera and launch its thread
//   LOGFILE << "Launches the camera thread";
//   camera_thread_ptr_.reset(new camera::CameraThread);
//   camera_thread_ptr_->setLogDirectory(logDirectory);
//   LOGFILE << "Launched the camera thread";

//   // Launch the server's thread
//   int const port = 30000;
//   try {
//     LOGFILE << "Launching the server's thread";
//     server_thread_ptr_.reset(new network::ServerThread(port));
//     server_thread_ptr_->setCameraThread(camera_thread_ptr_.get());
//     LOGFILE << "Launched the server's thread";
//   } catch(network::SocketException const& e) {
//     CONSOLE << e.description();
//   }
// }

// //--------------------------------------------------------------------------------------------------

// Module::~Module(){}

// //--------------------------------------------------------------------------------------------------
// // Methods
// //--------------------------------------------------------------------------------------------------

// void Module::join()
// {
//   camera_thread_ptr_->join();
//   server_thread_ptr_->join();
// }

// //--------------------------------------------------------------------------------------------------

// } // namespace module
