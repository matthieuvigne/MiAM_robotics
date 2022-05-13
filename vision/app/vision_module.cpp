#include <iostream>

#include <common/logger.hpp>
#include <module/module.hpp>
#ifdef USE_TEST_BENCH
#include <testing/test_bench.hpp>
#endif

// Main routine
int main(int argc, char* argv[])
{
  // Initialize the logger
  //~ std::string const filename = "vision_module_logs.txt";
  //~ INIT_LOGGERS(filename);
  common::ConsoleLogger::init();

  // Initialize the test bench if required
  #ifdef USE_TEST_BENCH
  common::Team team = common::Team::PURPLE;
  testing::TestBench::Options options = testing::TestBench::Options::getDefaultOptions(team);
  options.mode = testing::TestBench::Mode::PERFECT;
  testing::TestBench::init(options);
  #endif

  // Initialize the vision module (launches the internal camera and server threads)
  module::Module::UniquePtr module_ptr(new module::Module);
  module_ptr->join();
  return EXIT_SUCCESS;
}
