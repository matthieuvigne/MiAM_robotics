#include <common/logger.hpp>
#include <network/camera_client.hpp>

int main(int argc, char* argv[])
{
  common::ConsoleLogger::init();
  network::CameraClient client;
  CONSOLE << "Initializing the camera client...";
  client.run();
  return EXIT_SUCCESS;
}
