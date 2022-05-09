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
      condition_.wait(locker, [&](){ return (team_ != common::Team::UNKNOWN); });
      CameraPoseFilter::Params filter_params = CameraPoseFilter::Params::getDefaultParams(team_);
      pose_filter_ptr_.reset(new CameraPoseFilter(filter_params));
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
    common::MarkerList detected_markers;
    #if USE_TEST_BENCH
    TEST_BENCH_PTR->detectMarkers(&detected_markers);
    #else
    cv::Mat image;
    camera_ptr_->takePicture(image);
    camera_ptr_->detectMarkers(image, &detected_markers);
    #endif

    // Check if the central marker is detected => if so: update the filter
    common::MarkerList::iterator it = detected_markers.begin();
    for(it; it != detected_markers.end(); ++it)
    {
      if(it->getId() == 42)
      {
        // Covariances have already been converted from radians to degrees
        Eigen::Affine3d const* TCM = it->getTCM();
        Eigen::Matrix<double,6,6> const* cov_TCM = it->getCovTCM();
        pose_filter_ptr_->update(*TCM, *cov_TCM);
      }
    }

    // Update the pose estimate of all the markers
    // [TODO] Replace with a function
    //~ eraseOldMarkerEstimates();
    Eigen::Affine3d const& TWC = pose_filter_ptr_->getTWC();
    Eigen::Matrix<double,6,6> const cov_TWC = pose_filter_ptr_->getCovTWC();
    for(it = detected_markers.begin(); it != detected_markers.end(); ++it)
    {
      common::Marker& detected_marker = *it;
      detected_marker.estimateFromCameraPose(TWC, cov_TWC);
      std::lock_guard<std::mutex> const lock(mutex_);
      //~ marker_estimates_.insert(std::make_pair(marker.timestamp_ns, marker));
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

void CameraThread::eraseOldMarkerEstimates()
{
  //~ // Get the current timestamp
  //~ int64_t constexpr max_duration_ns = 1e9;
  //~ int64_t const timestamp_now_ns = common::convertToNanoseconds(common::Time::now());
  //~ int64_t const timestamp_old_ns = timestamp_now_ns - max_duration_ns;

  //~ // Erase the old markers
  //~ std::lock_guard<std::mutex> lock(mutex_);
  //~ common::MarkerEstimates::iterator it_begin = marker_estimates_.begin();
  //~ common::MarkerEstimates::iterator it_end = marker_estimates_.upper_bound(timestamp_old_ns);
  //~ marker_estimates_.erase(it_begin, it_end);
}

//--------------------------------------------------------------------------------------------------

void CameraThread::getMarkers(common::MarkerEstimates* estimates_ptr) const
{
  //~ common::MarkerEstimates& estimates = *estimates_ptr;
  //~ estimates.clear();
  //~ std::lock_guard<std::mutex> lock(mutex_);
  //~ estimates.insert(marker_estimates_.cbegin(), marker_estimates_.cend());
}

//--------------------------------------------------------------------------------------------------

void CameraThread::setTeam(common::Team team) const
{
  mutex_.lock();
  team_ = team;
  mutex_.unlock();
  condition_.notify_one();
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
