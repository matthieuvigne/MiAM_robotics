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

bool TestBench::takePicture(int camera_azimuth_deg, cv::Mat* image_ptr)
{
  try
  {
    // Get the reference to the image
    CHECK_NOTNULL(image_ptr);
    cv::Mat& image = *image_ptr;
    
    // Get the string for the azimuth angle
    std::vector<char> azimuth_char(4);
    std::snprintf(azimuth_char.data(), 4, "%+03d", camera_azimuth_deg);
    std::string const azimuth_str(azimuth_char.data(), 3); // ignore termination character
    
    // Get the name of the image
    std::string filename = image_folder_ + "/azimuth_" + azimuth_str + ".jpg";
    image = cv::imread(filename, cv::IMREAD_COLOR);
  }
  catch(std::exception const&)
  {
    return false;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace testing
