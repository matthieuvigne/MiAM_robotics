#include <iostream>

// Include the filesystem library
#ifndef __has_include
  static_assert(false, "__has_include not supported");
#else
#  if __cplusplus >= 201703L && __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = fs;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif

#include <common/logger.hpp>
#include <module/module.hpp>
#ifdef USE_TEST_BENCH
#include <testing/test_bench.hpp>
#endif

// Main routine
int main(int argc, char* argv[])
{
  // Create new directory for this run
  std::string const rootDir = "/home/pi/vision_module_log";
  fs::space_info tmp = fs::space(rootDir);
  
  long int MIN_DISK_SPACE = 1e9;
  if (tmp.free < MIN_DISK_SPACE)
  {
    std::cout << "Disk almost full - removing old logs" << std::endl;
    fs::remove_all(rootDir);
    fs::create_directory(rootDir);
  }

  int dirId = 0;
  while (fs::exists((rootDir + "/log" + std::to_string(dirId))))
    dirId ++;
  std::string const dirPath = rootDir + "/log" + std::to_string(dirId);
  fs::create_directory(dirPath);

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
