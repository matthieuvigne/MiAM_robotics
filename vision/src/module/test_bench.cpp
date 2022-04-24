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

  // Initialize the transformation from the reference frame to the camera frame
  Eigen::Affine3d const TRC =
      Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  TRC_ = common::PoseSampler::sample(TRC, 1.0*RAD, 1e-3);

  // Initialize the central marker
  common::MarkerId const central_marker_id =
    common::Marker::sampleMarkerId(common::MarkerFamily::CENTRAL_MARKER);
  Eigen::Affine3d TWM =
      Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  TWM = common::PoseSampler::sample(TWM, 1.0*RAD, 5e-3);
  markers_.emplace(central_marker_id, TWM);

  // Add markers for random samples on the field
  int constexpr num_samples = 10;
  for(int sample_idx=0; sample_idx<num_samples; sample_idx++)
  {
    common::MarkerId const marker_id =
      common::Marker::sampleMarkerId(common::MarkerFamily::ROCK_SAMPLE);
    Eigen::Affine3d TWS =
      Eigen::Translation3d(0.,0.,0.)
      * Eigen::AngleAxisd(0.,Eigen::Vector3d::UnitZ());
    //~ TWS = common::PoseSampler::sample(TWS, 
  }

  // Sample the robots and marker poses
  // [TODO]

  // Set the initialization indicator
  is_initialized_ = true;
}

//--------------------------------------------------------------------------------------------------

void rotateCamera(double wx, double wy, double wz)
{
  CHECK(is_initialized_);
  Eigen::Matrix<double,6,1> tau;
  tau << wx, wy, wz, 0.0, 0.0, 0.0;
  Eigen::Affine3d const TRkRkp1 = common::so3r3::expMap(tau);
  TWC_ = TWC_ * TRC_.inverse() * TRkRkp1 * TRC_;
}

//--------------------------------------------------------------------------------------------------

void detectMarkers(common::DetectedMarkerList* detected_markers_ptr)
{
  // Get the reference to the detected markers
  CHECK(is_initialized_);
  CHECK_NOTNULL(detected_markers_ptr);
  common::DetectedMarkerList& detected_markers = *detected_markers_ptr;

  // For each marker, check whether it is visible and get its estimate
  // Test1: report all the markers with simple covariance independently of their visibility
  // Test2: check the visibility of the markers and compute their "true" associated covariance
  // [TODO]
}

//--------------------------------------------------------------------------------------------------

void setCameraRotationProcessNoise(double camera_sigma_w)
{
  CHECK(camera_sigma_w >= 0);
  camera_sigma_w_ = camera_sigma_w;
}

//--------------------------------------------------------------------------------------------------

void setMarkerRotationMeasurementNoise(double marker_sigma_w)
{
  CHECK(marker_sigma_w >= 0);
  marker_sigma_w_ = marker_sigma_w;
}

//--------------------------------------------------------------------------------------------------

void setMarkerPositionMeasurementNoise(double marker_sigma_t)
{
  CHECK(marker_sigma_t >= 0);
  marker_sigma_t_ = marker_sigma_t;
}

//--------------------------------------------------------------------------------------------------

} // namespace test

} // namespace module
