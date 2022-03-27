#ifndef VISION_CAMERA_THREAD_HPP
#define VISION_CAMERA_THREAD_HPP

#include <mutex>
#include <queue>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>
#include <vision/camera.hpp>
#include <vision/camera_pose_filter.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraThread {

public:

  POINTER_TYPEDEF(CameraThread);
  CameraThread(
    Eigen::Affine3d const& T_WM,
    Eigen::Affine3d const& T_RC,
    Eigen::Matrix<double,6,6> const& cov_T_RC,
    Camera::UniquePtr camera);
  virtual ~CameraThread();

public:

  inline void join();
  void getMarkerEstimates(common::MarkerIdToEstimate* estimates) const;

private:

  void runThread();
  void getMeasurements() const;
  void rotateCameraToAnglePosition(double angle_deg);
  void incrementCameraAngle(double& camera_angle, double delta_angle);

private:

  // Camera and pose estimation filter
  Eigen::Affine3d T_WM_; ///< Relative pose from world to central marker
  Eigen::Affine3d T_RC_; ///< Relative pose from reference to camera
  Eigen::Matrix<double,6,6> cov_T_RC_; /// Relative pose covariance
  Camera::UniquePtr camera_ptr_; ///< Camera
  CameraPoseFilter::UniquePtr pose_filter_ptr_; ///< Camera pose filter

  // Thread
  mutable std::mutex mutex_;
  common::MarkerIdToEstimate marker_id_to_estimate_;
  std::unique_ptr<std::thread> thread_ptr_;

}; // class CameraThread

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

void CameraThread::join()
{
  this->thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_CAMERA_THREAD_HPP
