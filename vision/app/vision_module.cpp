#include <iostream>

#include <common/logger.hpp>
#include <module/module.hpp>
#ifdef USE_TEST_BENCH
#include <module/test_bench.hpp>
#endif

// Main routine
int main(int argc, char* argv[])
{
  // Initialize the logger
  //~ std::string const filename = "vision_module_logs.txt";
  //~ INIT_LOGGERS(filename);
  common::ConsoleLogger::init();
  CONSOLE << "Loggers have been initialized.";

  // Initialize the test bench if required
  #ifdef USE_TEST_BENCH
  CONSOLE << "Initializing the test bench";
  module::TestBench::Options options = module::TestBench::Options::getDefaultOptions();
  options.mode = module::TestBench::Mode::PERFECT;
  module::TestBench::init(options);
  CONSOLE << "Test bench has been initialized.";
  #endif

  // Initialize the vision module (launches the internal camera and server threads)
  CONSOLE << "Module initialization";
  module::ModuleParams parameters = module::ModuleParams::getDefaultParams();
  module::Module::UniquePtr module_ptr(new module::Module(parameters));
  CONSOLE << "Module has been initialized.";
  module_ptr->join();
  return EXIT_SUCCESS;
}
