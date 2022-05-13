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
  // Add team option
  Options options;
  options.team = team;
  options.TWC = Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  options.TRC = Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  options.TWM = common::getTWM();

  return options;
}

//--------------------------------------------------------------------------------------------------

void TestBench::init(Options const& options)
{
  test_bench_ptr.reset(new TestBench(options));
}

//--------------------------------------------------------------------------------------------------

TestBench::TestBench(Options const& options)
{
  // Check this is the only test bench
  if(is_initialized_)
    throw std::runtime_error("Cannot initialize multiple test benches");
  is_initialized_ = true;

  // Initialize the transformation from the world frame to the camera frame
  TWC_ = options.TWC;
  if(options.mode == Mode::NOISY)
    TWC_ = testing::PoseGaussianSampler::sample(TWC_, options.TWCi_sigma_w, options.TWCi_sigma_t);

  // Initialize the transformation from the reference frame to the camera frame
  TRC_ = options.TRC;
  if(options.mode == Mode::NOISY)
    TRC_ = testing::PoseGaussianSampler::sample(TRC_, options.TRC_sigma_w, options.TRC_sigma_t);

  // Initialize the central marker
  common::Marker::Id const central_marker_id =
    common::Marker::sampleMarkerId(common::Marker::Family::CENTRAL_MARKER);
  Eigen::Affine3d TWM = options.TWM;
  if(options.mode == Mode::NOISY)
    TWM = testing::PoseGaussianSampler::sample(TWM, options.TWM_sigma_w, options.TWM_sigma_t);
  //~ markers_.emplace(central_marker_id, TWM);

  // Sample the other sample markers
  testing::PoseUniformSampler const marker_sampler(
    Eigen::Vector3d{-1.0*RAD,-1.0*RAD,-M_PI}, Eigen::Vector3d{1.0*RAD,1.0*RAD,M_PI},
    Eigen::Vector3d{0.,0.,0.}, Eigen::Vector3d{board_width_,board_height_,0.});
  int constexpr num_samples = 10;
  for(int sample_idx=0; sample_idx<num_samples; sample_idx++)
  {
    common::Marker::Id const marker_id =
      common::Marker::sampleMarkerId(common::Marker::Family::ROCK_SAMPLE);
    Eigen::Affine3d const TWS = marker_sampler.sample();
    //~ markers_.emplace(marker_id, TWS);
  }
  
  // Set the process and measurement noise parameters
  camera_sigma_w_ = options.camera_sigma_w;
  marker_sigma_w_ = options.marker_sigma_w;
  marker_sigma_t_ = options.marker_sigma_t;
}

//--------------------------------------------------------------------------------------------------

void TestBench::rotateCamera(double dtheta_deg)
{
  CHECK(is_initialized_);
  Eigen::Matrix<double,6,1> tau;
  tau << 0.0, dtheta_deg*RAD, 0.0, 0.0, 0.0, 0.0;
  Eigen::Affine3d const TRkRkp1 = common::so3r3::expMap(tau);
  TWC_ = TWC_ * TRC_.inverse() * TRkRkp1 * TRC_;
}

//--------------------------------------------------------------------------------------------------

void TestBench::rotateCameraToAnglePosition(double angle_deg)
{
  CHECK(is_initialized_);
  Eigen::Affine3d TWR = TWC_ * TRC_.inverse();
  Eigen::Quaterniond const qWR = 
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
  TWC_ = Eigen::Translation3d(TWR.translation()) * qWR * TRC_;
}

//--------------------------------------------------------------------------------------------------

void TestBench::detectMarkers(common::MarkerList* detected_markers_ptr, Mode mode)
{
  // Get the reference to the detected markers
  CHECK(is_initialized_);
  CHECK_NOTNULL(detected_markers_ptr);
  common::MarkerList& detected_markers = *detected_markers_ptr;
  detected_markers.clear();

  // For each marker, check whether it is visible and get its estimate
  int64_t const timestamp_ns = common::convertToNanoseconds(common::Time::now());
  //~ for(common::MarkerIdToPose::value_type const& pair : markers_)
  //~ {
    //~ // Check if the marker is visible
    //~ // [TODO]

    //~ // Simulate the observation of the marker
    //~ common::MarkerId const marker_id = pair.first;
    //~ common::Marker marker(marker_id);
    //~ Eigen::Affine3d const& TWM = pair.second;
    //~ Eigen::Affine3d TCM = TWC_.inverse() * TWM;
    //~ switch(mode)
    //~ {
      //~ case Mode::PERFECT:
        //~ break;
      //~ case Mode::NOISY:
        //~ TCM = common::PoseGaussianSampler::sample(TCM, marker_sigma_w_, marker_sigma_t_);
        //~ break;
      //~ default:
        //~ throw std::runtime_error("Unknown measurement mode");
    //~ }
    //~ Eigen::Matrix<double,6,6> cov_TCM = Eigen::Matrix<double,6,6>::Identity();
    //~ cov_TCM.block<3,3>(0,0) *= std::pow(marker_sigma_w_,2.0);
    //~ cov_TCM.block<3,3>(3,3) *= std::pow(marker_sigma_t_,2.0);

    //~ // Add the measurement to the marker
    //~ std::vector<cv::Point2f> corners(4);
    //~ marker.addMeasurement(timestamp_ns, corners, TCM, cov_TCM);
    //~ detected_markers.push_back(marker);
  //~ }
}

//--------------------------------------------------------------------------------------------------

void TestBench::setCameraRotationProcessNoise(double camera_sigma_w)
{
  CHECK(camera_sigma_w >= 0);
  camera_sigma_w_ = camera_sigma_w;
}

//--------------------------------------------------------------------------------------------------

void TestBench::setMarkerRotationMeasurementNoise(double marker_sigma_w)
{
  CHECK(marker_sigma_w >= 0);
  marker_sigma_w_ = marker_sigma_w;
}

//--------------------------------------------------------------------------------------------------

void TestBench::setMarkerPositionMeasurementNoise(double marker_sigma_t)
{
  CHECK(marker_sigma_t >= 0);
  marker_sigma_t_ = marker_sigma_t;
}

//--------------------------------------------------------------------------------------------------

double TestBench::getCameraRotationTime(double angle_rad)
{
  return angle_rad / camera_angular_speed_;
}

//--------------------------------------------------------------------------------------------------

} // namespace testing
