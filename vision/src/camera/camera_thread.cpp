#include <miam_utils/raspberry_pi/RPiGPIO.h>

#include <common/logger.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>
#include <common/time.hpp>
#include <camera/camera_thread.hpp>

#ifdef USE_TEST_BENCH
#include <testing/test_bench.hpp>
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

  struct timespec st, ct;
  clock_gettime(CLOCK_MONOTONIC, &st);
  ct = st;
  int image_id = 0;
  while(true)
  {

    struct timespec loopStart, currentTime;
    clock_gettime(CLOCK_MONOTONIC, &loopStart);

    // Initialize the filter if needed
    LOGFILE << "Initializing the filter";
    initializeFilterIfRequired();
    CHECK_NOTNULL(pose_filter_ptr_);
    LOGFILE << "Filter initializzed";

    // Take a picture and detect all the markers
    cv::Mat image;
    common::MarkerPtrList detected_markers;

    #if USE_TEST_BENCH
      TEST_BENCH_PTR->takePicture(camera_azimuth_deg_, &image);
      cv::imshow("Image", image);
      cv::waitKey(1000);
    #else
      camera_ptr_->takePicture(&image);
    #endif

    // Rotate the camera - do it now so the servo moves while we are analyzing the previous image.
    double const img_azimuth = camera_azimuth_deg_;
    double delta_azimuth_deg = (pose_filter_ptr_->isInitialized()) ? azimuth_step_deg_ : 0.0;
    delta_azimuth_deg = rotateCamera(delta_azimuth_deg);

    LOGFILE << "Detecting the markers";
    camera_ptr_->detectMarkers(image, img_azimuth, camera_elevation_deg_,
      &detected_markers, logDirectory_ + "/" + std::to_string(image_id));
    LOGFILE << "Deteceted the markers";


    LOGFILE << "####################################################";
    clock_gettime(CLOCK_MONOTONIC, &ct);
    double elapsedTime = (ct.tv_sec - st.tv_sec + (ct.tv_nsec - st.tv_nsec) / 1e9);
    LOGFILE << "Time:" << elapsedTime << " Image Id: " << image_id;
    LOGFILE << "   Azimuth " << img_azimuth << " estimated azimuth: " 
      << pose_filter_ptr_->getAzimuthDeg() << " estimated elevation: " << pose_filter_ptr_->getElevationDeg();
    image_id += 1;

    LOGFILE << "Detected markers: " << detected_markers.size();
    for (auto const& m : detected_markers)
    {
      LOGFILE << "Marker" << static_cast<int>(m->getId()) << "    TCM" << m->getTCM()->translation().transpose();
    }

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
        LOGFILE << "Camera pose updated";
        LOGFILE << "   Estimated position" << pose_filter_ptr_->getTWC().translation().transpose();
        LOGFILE << "   Filter innovation" << pose_filter_ptr_->getLastInnovation().transpose();

        // Update the camera azimuth and elevation estimates
        //~ camera_azimuth_deg_ = pose_filter_ptr_->getAzimuthDeg();
        //~ camera_elevation_deg_ = pose_filter_ptr_->getElevationDeg();
      }
    }
    // Remove multiple markers which are visible from the camera and add the new markers
    std::lock_guard<std::mutex> const lock(mutex_);
    Eigen::Affine3d const TRC = common::getTRC(img_azimuth, camera_elevation_deg_);
    Eigen::Affine3d const TCR = TRC.inverse();
    size_t const num_removed_markers = markers_->forEachMultipleMarkerRemoveIf(
      [&](common::Marker const& marker)
      {
        Eigen::Vector3d const& RuM = marker.getRuM();
        Eigen::Vector3d const CuM = TCR * RuM;
        Eigen::Vector2d IpM;
        double result = camera_ptr_->project(CuM, &IpM, 0);
        bool remove = result < 0.7;
        if (remove)
        {
            LOGFILE << "Removing marker in field of view:" << static_cast<int>(marker.getId())
              << "    TWM" << marker.getTWM()->translation().transpose();
        }
        return remove;
      }
    );
    Eigen::Affine3d const& TWC = pose_filter_ptr_->getTWC();
    LOGFILE << "   TWC" <<  TWC.matrix();
    LOGFILE << "   TRC" <<  TRC.matrix();
    Eigen::Matrix<double,6,6> const cov_TWC = pose_filter_ptr_->getCovTWC();
    for(it = detected_markers.begin(); it != detected_markers.end(); ++it)
    {
      common::Marker::UniquePtr& detected_marker_ptr = *it;
      detected_marker_ptr->estimateFromCameraPose(TWC, cov_TWC);

      LOGFILE << "Adding marker:" << static_cast<int>(detected_marker_ptr->getId()) << "    TWM" << detected_marker_ptr->getTWM()->translation().transpose();
      markers_->addMarker(std::move(detected_marker_ptr));
    }

    LOGFILE << "Current markers in store:";
    markers_->forEachMarker([&](common::Marker const& marker)
    {
      LOGFILE << "Marker" << static_cast<int>(marker.getId());
      LOGFILE << "    TCM" << marker.getTCM()->translation().transpose();
      LOGFILE << "    TWM" << marker.getTWM()->translation().transpose();
    });

    pose_filter_ptr_->predict(delta_azimuth_deg);
    // Now we sleep the remaining loop time.
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    double const dt = (currentTime.tv_sec - loopStart.tv_sec + (currentTime.tv_nsec - loopStart.tv_nsec) / 1e9);
    std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.01, 0.25 - dt)));
  }
}

//--------------------------------------------------------------------------------------------------

void CameraThread::rotateCameraToAnglePosition(double new_azimuth_deg)
{
  // Angle must be provided in degrees
  // Get the corresponding signal value
  // Angle -90°: camera at position 500
  // Angle   0°: camera at position 1500
  // Angle +90°: camera at position 2500
  double coeff = new_azimuth_deg / 90.0;
  coeff = std::max(-1.0,std::min(coeff,1.0));
  int const signal = 1400 + 1000 * coeff; // 1400: offset to have the camera straight

  // Send the new position order to the servo
  std::string const command = "echo " + std::to_string(1000 * signal)
    + " > /sys/class/pwm/pwmchip0/pwm0/duty_cycle";
  system(command.c_str());
}

//--------------------------------------------------------------------------------------------------

double CameraThread::rotateCamera(double delta_azimuth_deg)
{
  // Get the new azimuth angle in degrees
  static bool increasing_angle = true;
  double const min_azimuth_deg = -max_azimuth_deg_;
  double const max_azimuth_deg =  max_azimuth_deg_;
  double new_azimuth_deg = increasing_angle
    ? camera_azimuth_deg_ + delta_azimuth_deg
    : camera_azimuth_deg_ - delta_azimuth_deg;
  if(new_azimuth_deg >= max_azimuth_deg)
  {
    increasing_angle = false;
    new_azimuth_deg = max_azimuth_deg;
  }
  else if(new_azimuth_deg <= min_azimuth_deg)
  {
    increasing_angle = true;
    new_azimuth_deg = min_azimuth_deg;
  }

  // Move the camera to this new azimuth angle
  #if !USE_TEST_BENCH
    rotateCameraToAnglePosition(new_azimuth_deg);
  #endif
  delta_azimuth_deg = new_azimuth_deg - camera_azimuth_deg_;
  camera_azimuth_deg_ = new_azimuth_deg;
  return delta_azimuth_deg;
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
