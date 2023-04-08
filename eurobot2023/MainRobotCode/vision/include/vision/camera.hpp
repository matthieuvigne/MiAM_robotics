#ifndef CAMERA_CAMERA_HPP
#define CAMERA_CAMERA_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <common/macros.hpp>
#include <vision/marker.hpp>
#include <vision/distortion_model.hpp>

#ifdef RPI4
  #include "LibCamera.h"
#else
  #include <raspicam/raspicam_cv.h>
#endif

namespace vision {

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Camera {

public:

  POINTER_TYPEDEF(Camera);
  struct Intrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
  };

  struct Params {
    std::string name;
    enum {WIDTH, HEIGHT};
    std::array<double,2> resolution;
    enum {FX, FY, CX, CY};
    std::array<double,4> intrinsics;
    DistortionModel::Type distortion_model;
    std::vector<double> distortion_coeffs;
    Eigen::Affine3d pose;
    static Params getDefaultParams();
  }; // struct CameraParams

public:

  DISALLOW_EVIL_CONSTRUCTORS(Camera);
  Camera(Params const& params);
  virtual ~Camera() = default;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  // Getters
  inline uint32_t getImageWidth() const;
  inline uint32_t getImageHeight() const;
  inline DistortionModel::Type getDistortionModel() const;
  void getCameraMatrix(Eigen::Matrix3d* matrix) const;
  void getCameraMatrix(cv::Mat* matrix) const;

  // Camera projection
  double project(
    Eigen::Vector3d const& point_3d,
    Eigen::Vector2d* point_2d,
    Eigen::Matrix<double,2,3>* out_jacobian) const;

  // Compute angles
  void backProject(
    Eigen::Vector2d const& point2d,
    Eigen::Vector3d* point3d) const;

  // Take picture and detect markers on it
  bool takePicture(cv::Mat* image, double timeout = 1.0);
  bool detectMarkers(cv::Mat const& image,
    double camera_azimuth_deg, double camera_elevation_deg,
    MarkerPtrList* detected_markers_ptr,
    std::string const& imageLogPath) const;

private:
  void configureCamera();

private:

  // Camera name
  std::string const name_;

  // Camera properties
  std::mutex mutex_;
  uint32_t image_width_;
  uint32_t image_height_;
  Intrinsics intrinsics_;
  DistortionModel::UniquePtr distortion_;
  Eigen::Affine3d pose_;
  #ifdef RPI4
    LibCamera camera_;
    LibcameraOutData frameData_;
  #else
    raspicam::RaspiCam_Cv camera_handler_;
  #endif

  // Marker properties
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  bool isRunningOnRPi_; ///< Detect if running on RPi, otherwise disable functions.

}; // class Camera

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

uint32_t Camera::getImageWidth() const
{
  return image_width_;
}

//--------------------------------------------------------------------------------------------------

uint32_t Camera::getImageHeight() const
{
  return image_height_;
}

//--------------------------------------------------------------------------------------------------

DistortionModel::Type Camera::getDistortionModel() const
{
  return distortion_->getType();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // CAMERA_CAMERA_HPP
