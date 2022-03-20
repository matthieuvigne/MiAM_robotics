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

private:

  void runThread();
  void getMeasurements() const;

private:

  // Camera and pose estimation filter
  Eigen::Affine3d T_WM_; ///< Relative pose from world to central marker
  Eigen::Affine3d T_RC_; ///< Relative pose from reference to camera
  Eigen::Matrix<double,6,6> cov_T_RC_; /// Relative pose covariance
  Camera::UniquePtr camera_ptr_; ///< Camera
  CameraPoseFilter::UniquePtr pose_filter_ptr_; ///< Camera pose filter

  // Thread
  mutable std::mutex mtx_;
  std::unique_ptr<std::thread> thread_ptr_;

}; // class CameraThread

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_CAMERA_THREAD_HPP
