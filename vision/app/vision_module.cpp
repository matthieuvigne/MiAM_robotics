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
  std::string const filename = "log.txt";
  common::Logger::init(filename);
  LOG("Logger is initialized");
  LOGGER << "Test logging";
  LOGGER << "Test logging bis";
  return EXIT_SUCCESS;

  // Initialize the test bench if required
  #ifdef USE_TEST_BENCH
    module::TestBench::Mode test_mode = module::TestBench::Mode::PERFECT;
    module::TestBench::initializeTestBench(test_mode);
    LOG("Test bench initialized");
  #endif

  // Initialize the vision module (launches the internal camera and server threads)
  module::ModuleParams parameters = module::ModuleParams::getDefaultParams();
  module::Module::UniquePtr module_ptr(new module::Module(parameters));
  module_ptr->join();
  return EXIT_SUCCESS;
}
