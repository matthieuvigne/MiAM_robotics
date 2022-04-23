#include <common/pose_sampler.hpp>
#include <module/test_bench.hpp>

namespace module {

namespace test {

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

void initializeTestBench()
{
  // Initialize the transformation from the world frame to the camera frame
  Eigen::Affine3d const TWC =
      Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  TWC_ = common::PoseSampler::sample(TWC, 3.0*RAD, 1e-2);

  // Initialize the transformation from the world frame to the central marker's frame
  Eigen::Affine3d const TWM =
      Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  TWM_ = common::PoseSampler::sample(TWM, 1.0*RAD, 5e-3);

  // Initialize the transformation from the reference frame to the camera frame
  Eigen::Affine3d const TRC =
      Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  TRC_ = common::PoseSampler::sample(TRC, 1.0*RAD, 1e-3);

  // Sample the robots and marker poses
  // [TODO]
}

//--------------------------------------------------------------------------------------------------

void rotateCamera(double wx, double wy, double wz)
{
  Eigen::Matrix<double,6,1> tau;
  tau << wx, wy, wz, 0.0, 0.0, 0.0;
  Eigen::Affine3d const TRkRkp1 = common::so3r3::expMap(tau);
  TWC_ = TWC_ * TRC_.inverse() * TRkRkp1 * TRC_;
}

//--------------------------------------------------------------------------------------------------

void detectMarkers(common::DetectedMarkerList* detected_markers_ptr)
{
  // Get the reference to the detected markers
  CHECK_NOTNULL(detected_markers_ptr);
  common::DetectedMarkerList& detected_markers = *detected_markers_ptr;

  // For each marker, check whether it is visible and get its estimate
  // Test1: report all the markers with simple covariance independently of their visibility
  // Test2: check the visibility of the markers and compute their "true" associated covariance
  // [TODO]
}

//--------------------------------------------------------------------------------------------------

} // namespace test

} // namespace module
