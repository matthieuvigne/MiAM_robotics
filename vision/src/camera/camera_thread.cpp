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

CameraThread::CameraThread()
{
  camera::Camera::Params const camera_params = camera::Camera::Params::getDefaultParams();
  camera_ptr_.reset(new camera::Camera(camera_params));
  pose_filter_ptr_ = nullptr;
  thread_ptr_.reset(new std::thread([=](){runThread();}));
}

//--------------------------------------------------------------------------------------------------

CameraThread::~CameraThread()
{
  thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void CameraThread::runThread()
{
  // Rotate camera to the zero position
  camera_angle_deg_ = 0.0;
  #if USE_TEST_BENCH
  TEST_BENCH_PTR->rotateCameraToAnglePosition(camera_angle_deg_*RAD);
  #else
  rotateCameraToAnglePosition(camera_angle_deg_);
  #endif

  // Initialize the camera pose filter and get the measurements
  while(true)
  {
    // Check if the client has sent an initialization message
    if(pose_filter_ptr_ == nullptr)
    {
      std::unique_lock<std::mutex> locker(mutex_);
      condition_.wait(locker, [&](){ return (team_ptr_!=nullptr); });
      if(team_ptr_)
      {
        common::Team team = *std::move(team_ptr_);
        CameraPoseFilter::Params filter_params = CameraPoseFilter::Params::getDefaultParams(team);
        pose_filter_ptr_.reset(new CameraPoseFilter(filter_params));
      }
      locker.unlock();
    }
    CHECK_NOTNULL(pose_filter_ptr_);

    // Rotate the camera and propagate the pose camera filter
    double const dtheta_deg = (pose_filter_ptr_->isInitialized()) ? increment_angle_deg_ : 0.0;
    #if USE_TEST_BENCH
    TEST_BENCH_PTR->rotateCamera(dtheta_deg*RAD);
    double const cam_rotation_time_sec = TEST_BENCH_PTR->getCameraRotationTime(dtheta_deg*RAD);
    std::this_thread::sleep_for(std::chrono::duration<double>(cam_rotation_time_sec));
    #else
    incrementCameraAngle(camera_angle_deg_, dtheta_deg);
    #endif
    pose_filter_ptr_->predict(dtheta_deg);

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
        // Covariances have already been converted from radians to degrees
        Eigen::Affine3d const T_CM = it->T_CM;
        Eigen::Matrix<double,6,6> const cov_TCM = it->cov_T_CM;
        pose_filter_ptr_->update(T_CM, cov_TCM);
      }
    }

    // Update the pose estimate of all the markers
    Eigen::Affine3d const& TWC = pose_filter_ptr_->getTWC();
    Eigen::Matrix<double,6,6> const cov_TWC = pose_filter_ptr_->getCovTWC();
    for(it = detected_markers.cbegin(); it != detected_markers.cend(); ++it)
    {
      common::DetectedMarker const& detected_marker = *it;
      common::Marker marker_estimate(TWC, cov_TWC, detected_marker);
      std::lock_guard<std::mutex> const lock(mutex_);
      marker_id_to_estimate_[marker_estimate.id] = marker_estimate;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CameraThread::rotateCameraToAnglePosition(double angle_deg)
{
  // Angle must be provided in degrees
  // Get the corresponding signal value
  // Angle -90°: camera at position 500
  // Angle   0°: camera at position 1500
  // Angle +90°: camera at position 2500
  double coeff = angle_deg / max_angle_deg_;
  coeff = std::max(-1.0,std::min(coeff,1.0));
  int const signal = 1500 + 1000*coeff;

  // Send the new position order to the servo
  std::string const command = "echo " + std::to_string(1000 * signal) + " > /sys/class/pwm/pwmchip0/pwm0/duty_cycle";
  system(command.c_str());
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
  TEST_BENCH_PTR->rotateCamera(true_delta_angle*RAD);
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
  std::lock_guard<std::mutex> lock(mutex_);
  estimates.insert(marker_id_to_estimate_.cbegin(), marker_id_to_estimate_.cend());
}

//--------------------------------------------------------------------------------------------------

void CameraThread::setTeam(common::Team team) const
{
  mutex_.lock();
  team_ptr_.reset(new common::Team(team));
  mutex_.unlock();
  condition_.notify_one();
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
