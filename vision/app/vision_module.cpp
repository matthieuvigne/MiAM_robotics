#include <iostream>

#include <module/module.hpp>

// Declaration of the buffers
std::string const config_filename =
  "/home/rodolphe/Programming/MiAM_robotics/vision/data/params.yaml";

// Main routine
int main(int argc, char* argv[])
{
  // Vision module
  // Camera thread: at each turn, increment the camera, except if reception of a specific request
  // Server thread: receive and process the client's requests
  module::Module::UniquePtr module_ptr(new module::Module(config_filename));
}
