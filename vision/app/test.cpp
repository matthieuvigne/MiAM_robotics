#include <iostream>
#include <thread>
#include <vector>

#include <common/logger.hpp>
#include <testing/test_bench.hpp>

int sign(int a)
{
  if(a<0) return -1;
  if(a>0) return  1;
  return 0;
}

int main(int argc, char* argv[])
{
  // Initialize the test bench
  common::ConsoleLogger::init();
  common::Team const team = common::Team::PURPLE;
  testing::TestBench::Options options = testing::TestBench::Options::getDefaultOptions(team);
  testing::TestBench::init(options);

  // Get the camera images
  int max_iters = 50;
  int angle_step = 5;
  double camera_azimuth_deg = -30;
  for(int i=0; i<max_iters; i++)
  {
    cv::Mat image;
    TEST_BENCH_PTR->takePicture(camera_azimuth_deg, &image);
    cv::imshow("Image", image);
    cv::waitKey(500);
    if(std::abs(camera_azimuth_deg) == 30)
      angle_step = -sign(camera_azimuth_deg)*5;
    camera_azimuth_deg += angle_step;
  }

  //~ // Test the waiting function
  //~ while(true)
  //~ {
    //~ CONSOLE << "OK";
    //~ std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  //~ }

  return EXIT_SUCCESS;
}
