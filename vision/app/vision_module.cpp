#include <iostream>

#include <vision/module.hpp>

// Declaration of the buffers
std::string const config_filename = "/home/rodolphe/Programming/MiAM_robotics/vision/data/params.yaml";

// Main routine
int main(int argc, char* argv[])
{
  // Initialization of the vision module
  std::cout << "Initialization of the vision module" << std::endl;
  vision::Module::UniquePtr module_ptr(new vision::Module(config_filename));
  // -> initialization of the server
  // -> activation of the camera thread
  
  // Standard routine
  // -> todo
}
