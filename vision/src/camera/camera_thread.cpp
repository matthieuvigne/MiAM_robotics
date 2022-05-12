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
  std::lock_guard<std::mutex> lock(mutex_);
  markers_.reset(new common::MarkerStore);
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
  while(true)
  {
    // Initiliaze the filter if needed
    initializeFilterIfRequired();
    CHECK_NOTNULL(pose_filter_ptr_);

    // Rotate the camera and propagate the pose camera filter
    // rotateCameraToAngle
    double const dtheta_deg = (pose_filter_ptr_->isInitialized()) ? increment_angle_deg_ : 0.0;
    incrementCameraAngle(camera_azimuth_deg_, dtheta_deg);
    pose_filter_ptr_->predict(dtheta_deg);

    // Take a picture and detect all the markers
    cv::Mat image;
    common::MarkerPtrList detected_markers;
    camera_ptr_->takePicture(image);
    camera_ptr_->detectMarkers(image, camera_azimuth_deg_, camera_elevation_deg_, &detected_markers);

    // Check if the central marker is detected => if so: update the filter
    common::MarkerPtrList::iterator it = detected_markers.begin();
    for(it; it != detected_markers.end(); ++it)
    {
      common::Marker::UniquePtr& marker_ptr = *it;
      if(marker_ptr->isCentralMarker())
      {
        // Covariances have already been converted from radians to degrees
        Eigen::Affine3d const* TCM = marker_ptr->getTCM();
        Eigen::Matrix<double,6,6> const* cov_TCM = marker_ptr->getCovTCM();
        pose_filter_ptr_->update(*TCM, *cov_TCM);
      }
    }

    // Remove multiple markers which are visible from the camera and add the new markers
    std::lock_guard<std::mutex> const lock(mutex_);
    Eigen::Quaterniond const qRC = common::getqRC(camera_azimuth_deg_, camera_elevation_deg_);
    Eigen::Quaterniond const qCR = qRC.inverse();
    size_t const num_removed_markers = markers_->forEachMultipleMarkerRemoveIf(
      [&](common::Marker const& marker)
      {
        Eigen::Vector3d const& RuM = marker.getRuM();
        Eigen::Vector3d const CuM = qCR * RuM;
        Eigen::Vector2d IpM;
        camera::ProjectionResult const result = camera_ptr_->project(CuM, &IpM, 0);
        bool const remove = (result == camera::ProjectionResult::KEYPOINT_VISIBLE);
        return remove;
      }
    );
    Eigen::Affine3d const& TWC = pose_filter_ptr_->getTWC();
    Eigen::Matrix<double,6,6> const cov_TWC = pose_filter_ptr_->getCovTWC();
    for(it = detected_markers.begin(); it != detected_markers.end(); ++it)
    {
      common::Marker::UniquePtr& detected_marker_ptr = *it;
      detected_marker_ptr->estimateFromCameraPose(TWC, cov_TWC);
      markers_->addMarker(std::move(detected_marker_ptr));
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
  rotateCameraToAnglePosition(new_angle);
  camera_angle = new_angle;
}

//--------------------------------------------------------------------------------------------------

void CameraThread::getMarkers(common::MarkerList* markers_ptr) const
{
  markers_ptr->clear();
  common::MarkerList& markers = *markers_ptr;
  std::lock_guard<std::mutex> lock(mutex_);
  markers_->forEachMarker([&](common::Marker const& marker){ markers.emplace_back(marker);});
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

void CameraThread::initializeFilterIfRequired()
{
  bool initialize_filter = false;
  common::Team team = common::Team::UNKNOWN;

  // Wait for the first initialization of the filter
  std::unique_lock<std::mutex> locker(mutex_);
  if(pose_filter_ptr_ == nullptr)
  {
    condition_.wait(locker, [&](){ return (team_ != common::Team::UNKNOWN); });
    initialize_filter = true;
    team = team_;
  }
  else if(pose_filter_ptr_->getTeam() != team_)
  {
    initialize_filter = true;
    team = team_;
  }
  locker.unlock();

  // Initialize the filter if required
  if(initialize_filter)
  {
    camera_azimuth_deg_ = 0.;
    CameraPoseFilter::Params filter_params = CameraPoseFilter::Params::getDefaultParams(team_);
    pose_filter_ptr_.reset(new CameraPoseFilter(filter_params));
    rotateCameraToAnglePosition(camera_azimuth_deg_);
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
