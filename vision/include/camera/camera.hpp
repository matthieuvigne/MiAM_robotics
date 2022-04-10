#ifndef CAMERA_CAMERA_HPP
#define CAMERA_CAMERA_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <camera/distortion_model.hpp>
#include <camera/messages.hpp>

#include "LibCamera.h"

namespace camera {

//--------------------------------------------------------------------------------------------------
// Associated structures
//--------------------------------------------------------------------------------------------------

enum class ProjectionResult {
  KEYPOINT_VISIBLE,
  KEYPOINT_OUTSIDE_IMAGE,
  POINT_BEHIND_CAMERA,
  PROJECTION_INVALID,
  UNINITIALIZED
}; // ProjectionResult

struct CameraParams {
  std::string name;
  enum {WIDTH, HEIGHT};
  double resolution[2];
  enum {FX, FY, CX, CY};
  double intrinsics[4];
  DistortionModel::Type distortion_model;
  std::vector<double> distortion_coeffs;
  Eigen::Affine3d pose;
}; // struct CameraParams

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

public:

  DISALLOW_EVIL_CONSTRUCTORS(Camera);

  Camera(
    std::string const& name,
    uint32_t image_width, uint32_t image_height,
    double fx, double fy, double cx, double cy,
    DistortionModel::UniquePtr& distortion,
    Eigen::Affine3d const& pose);

  Camera(CameraParams const& params);

  virtual ~Camera() = default;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Camera::UniquePtr buildCameraFromYaml(
    std::string const& camera_name,
    YAML::Node const& camera_node);

public:

  // Getters
  inline uint32_t getImageWidth() const;
  inline uint32_t getImageHeight() const;
  inline DistortionModel const& getDistortionModel() const;
  void getCameraMatrix(Eigen::Matrix3d* matrix) const;
  void getCameraMatrix(cv::Mat* matrix) const;

  // Camera projection
  ProjectionResult project(
    Eigen::Vector3d const& point_3d,
    Eigen::Vector2d* point_2d,
    Eigen::Matrix<double,2,3>* out_jacobian) const;

  // Take picture
  bool takePicture(cv::Mat & image, double const& timeout = 1.0);

  // Get all detected markers with covariances
  bool detectMarkers(
    cv::Mat const& image,
    common::DetectedMarkerList* detected_markers) const;

private:
  void configureCamera();

  // Camera name
  std::string const name_;

  // Camera properties
  uint32_t image_width_;
  uint32_t image_height_;
  Intrinsics intrinsics_;
  DistortionModel::UniquePtr distortion_;
  Eigen::Affine3d pose_;

  LibCamera camera_;

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
  return this->image_width_;
}

//--------------------------------------------------------------------------------------------------

uint32_t Camera::getImageHeight() const
{
  return this->image_height_;
}

//--------------------------------------------------------------------------------------------------

DistortionModel const& Camera::getDistortionModel() const
{
  return *(this->distortion_);
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_HPP
