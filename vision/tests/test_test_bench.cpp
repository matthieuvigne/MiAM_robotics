#include <common/macros.hpp>
#include <module/test_bench.hpp>

//--------------------------------------------------------------------------------------------------
// Main routine
//--------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  // Initialize the test bench
  LOG("Initializing the test bench");
  module::TestBench::init(module::TestBench::Options::getDefaultOptions());
  LOG("Test bench initialized");

  // Get all the markers
  common::MarkerIdToPose const& markers = TEST_BENCH_PTR->getTrueMarkerPoses();
  LOG("Number of markers: " << markers.size() << std::endl);
  for(common::MarkerIdToPose::value_type const& v : markers)
  {
    common::MarkerId const& marker_id = v.first;
    Eigen::Affine3d const& marker_pose = v.second;
    Eigen::Vector3d const t = marker_pose.translation();
    LOG("ID: " << static_cast<int>(marker_id));
    LOG("T = " << t.transpose());
    CHECK( (t.x()>=0.) && (t.x()<=TEST_BENCH_PTR->board_width_));
    CHECK( (t.y()>=0.) && (t.y()<=TEST_BENCH_PTR->board_height_));
  }
  
  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
