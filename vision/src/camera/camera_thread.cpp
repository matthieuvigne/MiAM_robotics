#include <miam_utils/raspberry_pi/RPiGPIO.h>

#include <common/logger.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>
#include <common/time.hpp>
#include <camera/camera_thread.hpp>

#ifdef USE_TEST_BENCH
#include <module/test_bench.hpp>
#endif

namespace camera {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

CameraThread::CameraThread(
  Eigen::Affine3d const& T_WM,
  Eigen::Affine3d const& T_RC,
  Eigen::Matrix<double,6,6> const& cov_T_RC,
  Camera::UniquePtr camera_ptr)
: T_WM_       (T_WM),
  T_RC_       (T_RC),
  cov_T_RC_   (cov_T_RC),
  camera_ptr_ (std::move(camera_ptr))
{
  this->pose_filter_ptr_.reset(new CameraPoseFilter(T_WM, T_RC, cov_T_RC));
  this->thread_ptr_.reset(new std::thread([=](){this->runThread();}));
}

//--------------------------------------------------------------------------------------------------

CameraThread::~CameraThread()
{
  this->thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void CameraThread::runThread()
{
  // Initialization : scan the board looking for the central marker to initialize
  // ----------------------------------------------------------------------------
  LOGFILE << "Camera thread initialization";
  bool initialized = false;
  double camera_angle = 0.;
  while(!initialized)
  {
    // Detect the markers
    common::DetectedMarkerList detected_markers;
    #ifdef USE_TEST_BENCH
    TEST_BENCH_PTR->detectMarkers(&detected_markers, module::TestBench::Mode::PERFECT);
    LOGFILE << "Detected " << detected_markers.size() << " markers.";
    #else
    // Increment position and take picture
    cv::Mat image;
    rotateCameraToAnglePosition(camera_angle);
    camera_ptr_->takePicture(image);
    camera_ptr_->detectMarkers(image, &detected_markers);
    #endif

    // Check if the central marker (n°42) is detected
    common::DetectedMarkerList::const_iterator it = detected_markers.cbegin();
    for(it; it != detected_markers.cend(); ++it)
      if(it->marker_id == 42) break;

    // If so => initialize the camera pose filter and go to the next step
    if(it != detected_markers.cend())
    {
      // /!\ Protect the initialization of the filter
      LOGFILE << "Found the central marker";
      CHECK(static_cast<int>(it->marker_id) == 42);
      Eigen::Affine3d const T_CM = it->T_CM;
      Eigen::Matrix<double,6,6> const cov_T_CM = it->cov_T_CM;
      pose_filter_ptr_->setStateAndCovariance(CameraPoseFilter::InitType::T_CM, T_CM, cov_T_CM);
      LOGFILE << "Initialized the camera pose filter";
      break;
    }
  }
  return;

  // Routine : scan the board and detect all the markers
  // ---------------------------------------------------
  LOGFILE << "Camera thread's routine";
  while(true)
  {
    // Check if a specific request has been received
    // If so, realize it (don't forget to propagate the filter while doing it).
    // Rotate the camera and propagate the pose camera filter
    double const wy = increment_angle_deg_*RAD;
    double constexpr cov_wy = 1.0*RAD;
    #if USE_TEST_BENCH
    TEST_BENCH_PTR->rotateCamera(0.0, wy, 0.0);
    double const cam_rotation_time_sec = TEST_BENCH_PTR->getCameraRotationTime(wy);
    //~ LOG("Camera rotation time: " << cam_rotation_time_sec);
    std::this_thread::sleep_for(std::chrono::duration<double>(cam_rotation_time_sec));
    #else
    incrementCameraAngle(camera_angle, increment_angle_deg_);
    #endif
    pose_filter_ptr_->predict(wy, cov_wy, camera::CameraPoseFilter::Axis::Y);

    // Take a picture and detect all the markers
    common::DetectedMarkerList detected_markers;
    #if USE_TEST_BENCH
    TEST_BENCH_PTR->detectMarkers(&detected_markers);
    #else
    cv::Mat image;
    camera_ptr_->takePicture(image);
    camera_ptr_->detectMarkers(image, &detected_markers);
    #endif

    // Check if the central marker is detected => if so: update the filter
    common::DetectedMarkerList::const_iterator it = detected_markers.cbegin();
    for(it; it != detected_markers.cend(); ++it)
    {
      if(it->marker_id == 42)
      {
        Eigen::Affine3d const T_CM = it->T_CM;
        Eigen::Matrix<double,6,6> const cov_T_CM = it->cov_T_CM;
        this->pose_filter_ptr_->update(T_CM, cov_T_CM);
      }
    }

    // Update the pose estimate of all the markers
    Eigen::Affine3d const& T_WC = this->pose_filter_ptr_->getState();
    Eigen::Matrix<double,6,6> const& cov_T_WC = this->pose_filter_ptr_->getStateCovariance();
    for(it = detected_markers.cbegin(); it != detected_markers.cend(); ++it)
    {
      common::DetectedMarker const& detected_marker = *it;
      common::Marker marker_estimate(T_WC, cov_T_WC, detected_marker);
      std::lock_guard<std::mutex> const lock(this->mutex_);
      this->marker_id_to_estimate_[marker_estimate.id] = marker_estimate;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CameraThread::rotateCameraToAnglePosition(double angle)
{
  // Angle must be provided in degrees
  // Get the corresponding signal value
  // Angle -90: camera at position 500
  // Angle   0: camera at position 1500
  // Angle +90: camera at position 2500
  double coeff = angle / this->max_angle_deg_;
  coeff = std::max(-1.0,std::min(coeff,1.0));
  int const signal = 1500 + 1000*coeff;

  // Send the new position order to the servo
  // At 1200kHz, 24000 ticks = 20ms
  int constexpr channel = 0;
  RPi_setPWM(channel, static_cast<int>(1.2 * signal), 24000);
}

//--------------------------------------------------------------------------------------------------

void CameraThread::incrementCameraAngle(double& camera_angle, double delta_angle)
{
  // Get the new angle
  static bool increasing_angle = true;
  double const min_angle = -this->max_angle_deg_;
  double const max_angle =  this->max_angle_deg_;
  double new_angle = increasing_angle
    ? camera_angle + delta_angle
    : camera_angle - delta_angle;
  if(new_angle >= max_angle)
  {
    increasing_angle = false;
    new_angle = max_angle;
  }
  else if(new_angle <= min_angle)
  {
    increasing_angle = true;
    new_angle = min_angle;
  }

  // Move the camera to this new angle
  #if USE_TEST_BENCH
  double true_delta_angle = new_angle - camera_angle;
  TEST_BENCH_PTR->rotateCamera(0.0, true_delta_angle*RAD, 0.0);
  double const rot_time_sec = TEST_BENCH_PTR->getCameraRotationTime(true_delta_angle*RAD);
  std::this_thread::sleep_for(std::chrono::duration<double>(rot_time_sec));
  #else
  rotateCameraToAnglePosition(new_angle);
  #endif
  
  // Set the new camera angle
  camera_angle = new_angle;
}

//--------------------------------------------------------------------------------------------------

void CameraThread::getMarkers(common::MarkerIdToEstimate* estimates_ptr) const
{
  common::MarkerIdToEstimate& estimates = *estimates_ptr;
  estimates.clear();
  std::lock_guard<std::mutex> const lock(this->mutex_);
  estimates.insert(this->marker_id_to_estimate_.cbegin(), this->marker_id_to_estimate_.cend());
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
