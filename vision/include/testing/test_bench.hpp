#ifndef TESTING_TEST_BENCH_HPP
#define TESTING_TEST_BENCH_HPP

#include <common/common.hpp>
#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class TestBench {

public:
  struct Options;

public:
  POINTER_TYPEDEF(TestBench);
  TestBench(Options const& options);

public:
  static void init(Options const& options);
  bool takePicture(int camera_azimuth_deg, cv::Mat* image);

private:
  static bool is_initialized_;
  std::string const image_folder_;  

}; // class TestBench

//--------------------------------------------------------------------------------------------------

struct TestBench::Options {
  
  // Team
  common::Team team;
  std::string image_folder;

  // Functions
  static Options getDefaultOptions(common::Team team);

}; // struct TestBench::Options

//--------------------------------------------------------------------------------------------------
// Global variable declaration
//--------------------------------------------------------------------------------------------------

extern TestBench::UniquePtr test_bench_ptr;

//--------------------------------------------------------------------------------------------------

} // namespace testing

//--------------------------------------------------------------------------------------------------
// Global variable declaration
//--------------------------------------------------------------------------------------------------

#define TEST_BENCH_PTR testing::test_bench_ptr

//--------------------------------------------------------------------------------------------------

#endif // TESTING_TEST_BENCH_HPP
