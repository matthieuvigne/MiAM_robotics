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
  void getMarkers(common::MarkerList* markers) const;
  void setTeam(common::Team team) const;

private:

  void runThread();
  double rotateCamera(double delta_angle_deg);
  void rotateCameraToAnglePosition(double angle_deg);
  void initializeFilterIfRequired();

private:

  // Camera and pose estimation filter
  Camera::UniquePtr camera_ptr_;
  CameraPoseFilter::UniquePtr pose_filter_ptr_;

  // Camera rotation
  double camera_azimuth_deg_ = 0.0;
  double const camera_elevation_deg_ = 45.0;
  double const max_azimuth_deg_ = 30.0;
  #if USE_TEST_BENCH
    double const azimuth_step_deg_ = 5.0;
  #else
    double const azimuth_step_deg_ = 15.0;
  #endif

  // Thread and shared variables
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
