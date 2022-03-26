#ifndef VISION_CAMERA_HPP
#define VISION_CAMERA_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>

#include <common/macros.hpp>
#include <vision/distortion_model.hpp>
#include <vision/messages.hpp>

namespace vision {

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

struct DetectedMarker {
  int marker_id;
  Eigen::Affine3d T_CM;
  Eigen::Matrix<double,6,6> cov_T_CM;
}; // DetectedMarker
typedef std::vector<DetectedMarker> DetectedMarkerList;

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

  // Camera thread
  inline void launchThread();
  inline void join() const;
  inline void abortThread();

  // Camera projection
  ProjectionResult project(
    Eigen::Vector3d const& point_3d,
    Eigen::Vector2d* point_2d,
    Eigen::Matrix<double,2,3>* out_jacobian) const;

  // Get all detected markers with covariances
  bool detectMarkers(
    cv::Mat const& image,
    DetectedMarkerList* detected_markers) const;

private:

  static Eigen::Matrix3d skew(Eigen::Vector3d const& v);

  void cameraThread();

private:

  // Camera name
  std::string const name_;

  // Camera properties
  uint32_t image_width_;
  uint32_t image_height_;
  Intrinsics intrinsics_;
  DistortionModel::UniquePtr distortion_;
  Eigen::Affine3d pose_;

  // Marker properties
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

public:

  // Threading
  mutable std::mutex thread_mtx_;
  std::condition_variable thread_con_;
  vision_mgs::Image::UniquePtr thread_image_;
  std::unique_ptr<std::thread> thread_ptr_;
  bool abort_thread_ = false;

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

void Camera::launchThread()
{
  this->thread_ptr_.reset(new std::thread([=](){this->cameraThread();}));
}

//--------------------------------------------------------------------------------------------------

void Camera::join() const
{
  this->thread_ptr_->join();
}


//--------------------------------------------------------------------------------------------------

void Camera::abortThread()
{
  if(this->thread_ptr_ != nullptr)
  {
    this->abort_thread_ = true;
    this->thread_con_.notify_all();
    this->thread_ptr_->join();
    this->thread_ptr_ = nullptr;
    this->abort_thread_ = false;
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_CAMERA_HPP
