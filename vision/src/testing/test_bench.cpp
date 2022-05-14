#include <common/common.hpp>
#include <common/logger.hpp>
#include <testing/pose_gaussian_sampler.hpp>
#include <testing/pose_uniform_sampler.hpp>
#include <testing/test_bench.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Static and global variable definition
//--------------------------------------------------------------------------------------------------

bool TestBench::is_initialized_ = false;
TestBench::UniquePtr test_bench_ptr = nullptr;

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

TestBench::Options TestBench::Options::getDefaultOptions(common::Team team)
{
  Options options;
  options.team = team;
  options.image_folder = "/home/rodolphe/dev/src/MiAM_robotics/vision/data/img_rpi";
  return options;
}

//--------------------------------------------------------------------------------------------------

void TestBench::init(Options const& options)
{
  test_bench_ptr.reset(new TestBench(options));
}

//--------------------------------------------------------------------------------------------------

TestBench::TestBench(Options const& options)
: image_folder_ (options.image_folder)
{
  if(is_initialized_)
    throw std::runtime_error("Cannot initialize multiple test benches");
  is_initialized_ = true;
}

//--------------------------------------------------------------------------------------------------

bool TestBench::takePicture(double camera_azimuth_deg, cv::Mat* image_ptr)
{
  // Static variable counters
  static int test_img_idx = 0;
  static int azimuth_deg = int(camera_azimuth_deg);

  // Find the right image
  // TODO

  // Get the associated image
  CHECK_NOTNULL(image_ptr);
  cv::Mat& image = *image_ptr;
  //~ std::string const image_filename = image_folder_ + "/test" + test_img_idx_" + std::tostring("");
  //~ image = cv::imread(image_filename, cv::IMREAD_COLOR);
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace testing
