#ifndef CAMERA_CAMERA_THREAD_HPP
#define CAMERA_CAMERA_THREAD_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <common/common.hpp>
#include <common/macros.hpp>
#include <camera/camera.hpp>
#include <camera/camera_pose_filter.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraThread {

public:

  POINTER_TYPEDEF(CameraThread);
  struct Params {
    Eigen::Affine3d T_WM;
    Eigen::Affine3d T_RC;
    Eigen::Matrix<double,6,6> cov_T_RC;
    Camera::UniquePtr camera_ptr;
  }; // struct Params
  CameraThread(Params& params);
  virtual ~CameraThread();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  inline void join();
  void getMarkers(common::MarkerIdToEstimate* estimates) const;
  void setTeam(common::Team team) const;

private:

  void runThread();
  void rotateCameraToAnglePosition(double angle_deg);
  void incrementCameraAngle(double& camera_angle, double delta_angle);

private:

  // Camera and pose estimation filter
  Eigen::Affine3d T_WM_; ///< Relative pose from world to central marker
  Eigen::Affine3d T_RC_; ///< Relative pose from reference to camera
  Eigen::Matrix<double,6,6> cov_T_RC_; /// Relative pose covariance
  Camera::UniquePtr camera_ptr_; ///< Camera
  CameraPoseFilter::UniquePtr pose_filter_ptr_; ///< Camera pose filter

  // Camera rotation
  double camera_angle_deg_ = 0.0;
  double const max_angle_deg_ = 30.0;
  double const increment_angle_deg_ = 5.0;

  // Thread
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  mutable std::unique_ptr<common::Team> team_ptr_ = nullptr;
  common::MarkerIdToEstimate marker_id_to_estimate_;
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
