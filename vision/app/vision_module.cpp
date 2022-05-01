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
  //~ LOG("OK1");

  // Initialize the test bench if required
  #ifdef USE_TEST_BENCH
  LOGGER << "Test bench initialization";
  module::TestBench::Options options = module::TestBench::Options::getDefaultOptions();
  module::TestBench::init(options);
  LOGGER << "Test bench initialized";
  #endif
  //~ LOG("OK2");

  // Initialize the vision module (launches the internal camera and server threads)
  LOGGER << "Module initialization";
  module::ModuleParams parameters = module::ModuleParams::getDefaultParams();
  module::Module::UniquePtr module_ptr(new module::Module(parameters));
  module_ptr->join();
  return EXIT_SUCCESS;
}
