#include <iostream>

#include <common/logger.hpp>
#include <module/module.hpp>
#ifdef USE_TEST_BENCH
#include <testing/test_bench.hpp>
#endif

#include <filesystem>

// Main routine
int main(int argc, char* argv[])
{
  // Create new directory for this run
  std::string const rootDir = "/home/pi/vision_module_log";
  std::filesystem::space_info tmp = std::filesystem::space(rootDir);
  long int MIN_DISK_SPACE = 1e9;
  if (tmp.free < MIN_DISK_SPACE)
  {
    std::cout << "Disk almost full - removing old logs" << std::endl;
    std::filesystem::remove_all(rootDir);
    std::filesystem::create_directory(rootDir);
  }

  int dirId = 0;
  while (std::filesystem::exists((rootDir + "/log" + std::to_string(dirId))))
    dirId ++;
  std::string const dirPath = rootDir + "/log" + std::to_string(dirId);
  std::filesystem::create_directory(dirPath);

  // Initialize the logger
  common::ConsoleLogger::init();
  common::FileLogger::init(dirPath + "/vision_module.txt");

  // Initialize the test bench if required
  #ifdef USE_TEST_BENCH
  common::Team team = common::Team::PURPLE;
  testing::TestBench::Options options = testing::TestBench::Options::getDefaultOptions(team);
  testing::TestBench::init(options);
  #endif

  // Initialize the vision module (launches the internal camera and server threads)
  module::Module::UniquePtr module_ptr(new module::Module(dirPath));
  module_ptr->join();
  return EXIT_SUCCESS;
}
