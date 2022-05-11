#ifndef CAMERA_CAMERA_THREAD_HPP
#define CAMERA_CAMERA_THREAD_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <common/common.hpp>
#include <common/macros.hpp>
#include <common/marker_store.hpp>
#include <camera/camera.hpp>
#include <camera/camera_pose_filter.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraThread {

public:

  POINTER_TYPEDEF(CameraThread);
  CameraThread();
  virtual ~CameraThread();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  inline void join();
  void getMarkers(common::MarkerEstimates* estimates) const;
  void setTeam(common::Team team) const;

private:

  void runThread();
  void rotateCameraToAnglePosition(double angle_deg);
  void incrementCameraAngle(double& camera_angle, double delta_angle);
  void initializeFilterIfRequired();

private:

  // Camera and pose estimation filter
  Camera::UniquePtr camera_ptr_;
  CameraPoseFilter::UniquePtr pose_filter_ptr_;

  // Camera rotation
  double camera_azimuth_deg_ = 0.0;
  double const camera_elevation_deg_ = 45.0;
  double const max_angle_deg_ = 30.0;
  double const increment_angle_deg_ = 5.0;

  // Thread
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  mutable common::Team team_ = common::Team::UNKNOWN;
  common::MarkerStore::UniquePtr markers_;
  std::unique_ptr<std::thread> thread_ptr_;

}; // class CameraThread

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

void CameraThread::join()
{
  thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_THREAD_HPP
