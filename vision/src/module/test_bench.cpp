#include <common/logger.hpp>
#include <common/pose_gaussian_sampler.hpp>
#include <common/pose_uniform_sampler.hpp>
#include <module/test_bench.hpp>

namespace module {

//--------------------------------------------------------------------------------------------------
// Static and global variable definition
//--------------------------------------------------------------------------------------------------

bool TestBench::is_initialized_ = false;
TestBench::UniquePtr test_bench_ptr = nullptr;

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

TestBench::Options TestBench::Options::getDefaultOptions()
{
  // Add team option
  Options options;
  options.TWC = Eigen::Translation3d(1.5,2.0,1.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(3*M_PI_4, Eigen::Vector3d::UnitX());
  options.TRC = Eigen::Translation3d(0.0, 0.0, 0.0)
    * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY());
  options.TWM = Eigen::Translation3d(1.5,1.0,0.0)
    * Eigen::AngleAxisd();
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
  LOGFILE << "Test Bench initialization";

  // Check this is the only test bench
  if(is_initialized_)
    throw std::runtime_error("Cannot initialize multiple test benches");
  is_initialized_ = true;

  // Initialize the transformation from the world frame to the camera frame
  TWC_ = options.TWC;
  if(options.mode == Mode::NOISY)
    TWC_ = common::PoseGaussianSampler::sample(TWC_, options.TWCi_sigma_w, options.TWCi_sigma_t);
  LOGFILE << "TWC:\n" << TWC_.matrix();

  // Initialize the transformation from the reference frame to the camera frame
  TRC_ = options.TRC;
  if(options.mode == Mode::NOISY)
    TRC_ = common::PoseGaussianSampler::sample(TRC_, options.TRC_sigma_w, options.TRC_sigma_t);
  LOGFILE << "TRC:\n" << TRC_.matrix();

  // Initialize the central marker
  common::MarkerId const central_marker_id =
    common::Marker::sampleMarkerId(common::MarkerFamily::CENTRAL_MARKER);
  Eigen::Affine3d TWM = options.TWM;
  if(options.mode == Mode::NOISY)
    TWM = common::PoseGaussianSampler::sample(TWM, options.TWM_sigma_w, options.TWM_sigma_t);
  markers_.emplace(central_marker_id, TWM);
  LOGFILE << "TWM:\n" << TWM.matrix();

  // Sample the other sample markers
  common::PoseUniformSampler const marker_sampler(
    Eigen::Vector3d{-1.0*RAD,-1.0*RAD,-M_PI}, Eigen::Vector3d{1.0*RAD,1.0*RAD,M_PI},
    Eigen::Vector3d{0.,0.,0.}, Eigen::Vector3d{board_width_,board_height_,0.});
  int constexpr num_samples = 10;
  for(int sample_idx=0; sample_idx<num_samples; sample_idx++)
  {
    common::MarkerId const marker_id =
      common::Marker::sampleMarkerId(common::MarkerFamily::ROCK_SAMPLE);
    Eigen::Affine3d const TWS = marker_sampler.sample();
    markers_.emplace(marker_id, TWS);
    LOGFILE << "Marker with id " << static_cast<int>(marker_id) << " -> TWS:\n" << TWS.matrix();
  }
  
  // Set the process and measurement noise parameters
  camera_sigma_w_ = options.camera_sigma_w;
  marker_sigma_w_ = options.marker_sigma_w;
  marker_sigma_t_ = options.marker_sigma_t;
}

//--------------------------------------------------------------------------------------------------

void TestBench::rotateCamera(double wx, double wy, double wz)
{
  CHECK(is_initialized_);
  Eigen::Matrix<double,6,1> tau;
  tau << wx, wy, wz, 0.0, 0.0, 0.0;
  Eigen::Affine3d const TRkRkp1 = common::so3r3::expMap(tau);
  TWC_ = TWC_ * TRC_.inverse() * TRkRkp1 * TRC_;
}

//--------------------------------------------------------------------------------------------------

void TestBench::rotateCameraToAnglePosition(double angle_rad)
{
  CHECK(is_initialized_);
  Eigen::Affine3d TWR = TWC_ * TRC_.inverse();
  Eigen::Quaterniond const qWR = 
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
  TWC_ = Eigen::Translation3d(TWR.translation()) * qWR * TRC_;
}

//--------------------------------------------------------------------------------------------------

void TestBench::detectMarkers(common::DetectedMarkerList* detected_markers_ptr, Mode mode)
{
  // Get the reference to the detected markers
  CHECK(is_initialized_);
  CHECK_NOTNULL(detected_markers_ptr);
  common::DetectedMarkerList& detected_markers = *detected_markers_ptr;
  detected_markers.clear();

  // For each marker, check whether it is visible and get its estimate
  for(common::MarkerIdToPose::value_type const& pair : markers_)
  {
    // Check if the marker is visible
    // [TODO]

    // Simulate the observation of the marker
    common::DetectedMarker marker;
    marker.marker_id = pair.first;
    Eigen::Affine3d const& TWM = pair.second;
    Eigen::Affine3d const TCM = TWC_.inverse() * TWM;
    switch(mode)
    {
      case Mode::PERFECT:
        marker.T_CM = TCM;
        break;
      case Mode::NOISY:
        marker.T_CM = common::PoseGaussianSampler::sample(TCM, marker_sigma_w_, marker_sigma_t_);
        break;
      default:
        throw std::runtime_error("Unknown measurement mode");
    }
    marker.cov_T_CM.setIdentity();
    marker.cov_T_CM.block<3,3>(0,0) *= std::pow(marker_sigma_w_,2.0);
    marker.cov_T_CM.block<3,3>(3,3) *= std::pow(marker_sigma_t_,2.0);
    detected_markers.push_back(marker);
  }
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

} // namespace module
