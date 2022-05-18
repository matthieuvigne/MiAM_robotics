#include <common/common.hpp>
#include <common/maths.hpp>
#include <camera/camera.hpp>
#include <camera/distortion_fisheye.hpp>
#include <camera/distortion_null.hpp>
#include <camera/distortion_radtan.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Camera::Camera(Camera::Params const& params)
: name_             (params.name),
  pose_             (params.pose),
  dictionary_       (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250)),
  detector_params_  (cv::aruco::DetectorParameters::create())
{
  // Get the image resolution
  image_width_ = params.resolution[0];
  image_height_ = params.resolution[1];

  // Get the intrinsic parameters
  intrinsics_.fx = params.intrinsics[0];
  intrinsics_.fy = params.intrinsics[1];
  intrinsics_.cx = params.intrinsics[2];
  intrinsics_.cy = params.intrinsics[3];

  // Build the camera distortion model
  switch(params.distortion_model)
  {
    case DistortionModel::Type::NoDistortion:
    {
      distortion_ = DistortionModel::UniquePtr(new DistortionNull());
      break;
    }

    case DistortionModel::Type::RadTan:
    {
      Eigen::Map<Eigen::VectorXd const> const coeffs(params.distortion_coeffs.data(), 5u);
      distortion_ = DistortionModel::UniquePtr(new DistortionRadTan(coeffs));
      break;
    }

    case DistortionModel::Type::Fisheye:
    {
      Eigen::Map<Eigen::VectorXd const> const coeffs(params.distortion_coeffs.data(), 4u);
      distortion_ = DistortionModel::UniquePtr(new DistortionFisheye(coeffs));
      break;
    }

    default:
      throw std::runtime_error("Unrecognized distortion model.");
  }
  configureCamera();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Camera::getCameraMatrix(Eigen::Matrix3d* matrix_ptr) const
{
  CHECK_NOTNULL(matrix_ptr);
  Eigen::Matrix3d& matrix = (*matrix_ptr);
  matrix.setIdentity();
  matrix(0,0) = intrinsics_.fx;
  matrix(1,1) = intrinsics_.fy;
  matrix(0,2) = intrinsics_.cx;
  matrix(1,2) = intrinsics_.cy;
}

//--------------------------------------------------------------------------------------------------

void Camera::getCameraMatrix(cv::Mat* matrix_ptr) const
{
  CHECK_NOTNULL(matrix_ptr);
  cv::Mat& matrix = (*matrix_ptr);
  matrix = cv::Mat::eye(3, 3, CV_64FC1);
  matrix.at<double>(0,0) = intrinsics_.fx;
  matrix.at<double>(1,1) = intrinsics_.fy;
  matrix.at<double>(0,2) = intrinsics_.cx;
  matrix.at<double>(1,2) = intrinsics_.cy;
}

//--------------------------------------------------------------------------------------------------

ProjectionResult Camera::project(
  Eigen::Vector3d const& point_3d,
  Eigen::Vector2d* point_2d_ptr,
  Eigen::Matrix<double,2,3>* out_jacobian) const
{
  // Project the point onto the focal image plane
  CHECK_NOTNULL(point_2d_ptr);
  CHECK(point_3d.z() != 0.);
  Eigen::Vector2d& point_2d = *point_2d_ptr;
  point_2d(0) = point_3d.x() / point_3d.z();
  point_2d(1) = point_3d.y() / point_3d.z();

  // Apply distortion
  Eigen::Matrix2d J_Fxd_Fxu;
  if(out_jacobian)
    distortion_->distort(&point_2d, &J_Fxd_Fxu);
  else
    distortion_->distort(&point_2d, NULL);

  // Project onto the image plane
  double const& fx = intrinsics_.fx;
  double const& fy = intrinsics_.fy;
  double const& cx = intrinsics_.cx;
  double const& cy = intrinsics_.cy;
  double const xd = point_2d(0);
  double const yd = point_2d(1);
  point_2d(0) = fx*xd + cx;
  point_2d(1) = fy*yd + cy;

  // Check wheter the point is visible on the image or not
  bool is_visible = true;
  is_visible &= (point_2d(0) >= 0. && point_2d(0) <= image_width_);
  is_visible &= (point_2d(1) >= 0. && point_2d(1) <= image_height_);
  ProjectionResult result = is_visible
    ? ProjectionResult::KEYPOINT_VISIBLE
    : ProjectionResult::KEYPOINT_OUTSIDE_IMAGE;

  // Return the projection Jacobian matrix if required
  if(out_jacobian)
  {
    Eigen::Matrix2d J_Ixd_Fxd = Eigen::Matrix2d::Identity();
    J_Ixd_Fxd(0,0) = fx;
    J_Ixd_Fxd(1,1) = fy;

    Eigen::Matrix<double,2,3> J_Fxu_x3d = Eigen::Matrix<double,2,3>::Zero();
    J_Fxu_x3d.block<2,2>(0,0).setIdentity();
    J_Fxu_x3d(0,2) = - point_3d.x() / point_3d.z();
    J_Fxu_x3d(1,2) = - point_3d.y() / point_3d.z();
    J_Fxu_x3d *= 1. / point_3d.z();

    (*out_jacobian) = J_Ixd_Fxd * J_Fxd_Fxu * J_Fxu_x3d;
  }

  return result;
}

//--------------------------------------------------------------------------------------------------

bool Camera::detectMarkers(
  cv::Mat const& image,
  double camera_azimuth_deg,
  double camera_elevation_deg,
  common::MarkerPtrList* detected_markers_ptr
) const
{
  // Get all the markers on the image
  CHECK_NOTNULL(detected_markers_ptr);
  std::vector<int> detected_marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(image, dictionary_, marker_corners, detected_marker_ids,
    detector_params_, rejected_candidates);
  int64_t const timestamp_ns = common::convertToNanoseconds(common::Time::now());

  // If testing, show all the detected markers
  #if USE_TEST_BENCH
    cv::Mat image_with_markers = image.clone();
    cv::aruco::drawDetectedMarkers(image_with_markers, marker_corners, detected_marker_ids);
    cv::imshow("Image", image_with_markers);
    cv::waitKey(1000);
  #endif

  // Estimate the relative pose of the markers w.r.t. the camera
  // Marker corners are returned with the top-left corner first
  // Beware: the central marker has different dimensions than the regular markers
  cv::Mat camera_matrix, distortion_coeffs;
  distortion_->getDistortionCoeffs(&distortion_coeffs);
  getCameraMatrix(&camera_matrix);

  // Compute the covariance matrix for all detected markers
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6d;
  int const num_markers = detected_marker_ids.size();
  Eigen::Affine3d const TRC = common::getTRC(camera_azimuth_deg, camera_elevation_deg);
  for(int marker_idx=0; marker_idx<num_markers; ++marker_idx)
  {
    // Initialize the detected marker
    common::MarkerId const marker_id =
      static_cast<common::MarkerId>(detected_marker_ids[marker_idx]);
    common::Marker::UniquePtr marker_ptr(new common::Marker(marker_id));

    // Compute the direction of the beam
    int constexpr num_corners = 4;
    std::vector<cv::Point2f> const& corners = marker_corners[marker_idx];
    CHECK(corners.size() == num_corners);
    Eigen::Vector2d IpM = Eigen::Vector2d::Zero();
    for(cv::Point2f const& corner : corners) IpM += Eigen::Vector2d{corner.x,corner.y} / 4.;
    Eigen::Vector3d CuM;
    backProject(IpM, &CuM);
    Eigen::Vector3d const RuM = TRC * CuM;

    // Estimate the marker pose
    std::vector<cv::Vec3d> rvecs, tvecs;
    double const marker_length = marker_ptr->getSizeLength();
    std::vector<std::vector<cv::Point2f>> markers_corners{marker_corners[marker_idx]};
    cv::aruco::estimatePoseSingleMarkers(markers_corners, marker_length, camera_matrix,
      distortion_coeffs, rvecs, tvecs);
    Eigen::Map<Eigen::Vector3d> const CtCM((double*) tvecs[0].val);
    Eigen::Map<Eigen::Vector3d> const rvec((double*) rvecs[0].val);
    double const angle = rvec.norm();
    Eigen::Vector3d const axis = rvec.normalized();
    Eigen::Affine3d const TCM = Eigen::Translation3d(CtCM) * Eigen::AngleAxisd(angle,axis);

    // Update the image
    #if USE_TEST_BENCH
      cv::drawFrameAxes(image_with_markers, camera_matrix, distortion_coeffs,
        rvecs[0], tvecs[0], 0.1);
    #endif

    // Compute the measurement covariance matrix
    Eigen::Matrix<double,6,6> information_matrix = Eigen::Matrix<double,6,6>::Zero();
    for(int corner_idx=0; corner_idx<num_corners; ++corner_idx)
    {
      // Get the corner's coordinates w.r.t. the marker's frame
      Eigen::Vector3d MpCi;
      switch(corner_idx)
      {
        case 0: // top-left corner
          MpCi = Eigen::Vector3d(0,0,0);
          break;
        case 1: // top-right corner
          MpCi = Eigen::Vector3d(marker_length,0,0);
          break;
        case 2: // bottom-right corner
          MpCi = Eigen::Vector3d(marker_length,marker_length,0);
          break;
        case 3: // bottom-left corner
          MpCi = Eigen::Vector3d(0,marker_length,0);
          break;
      }

      // Project the 3d corner into the camera's frame
      Eigen::Vector3d const CpCi = TCM * MpCi;
      Eigen::Matrix<double,3,6> J_CpCi_TCM;
      J_CpCi_TCM.block<3,3>(0,0) = - common::skew(TCM.rotation()*MpCi);
      J_CpCi_TCM.block<3,3>(0,3).setIdentity();

      // Project the point onto the image plane
      Eigen::Vector2d IpCi;
      Eigen::Matrix<double,2,3> J_IpCi_CpCi;
      project(CpCi, &IpCi, &J_IpCi_CpCi);

      // Update the information matrix
      Eigen::Matrix<double,2,6> const J_IpCi_TCM = J_IpCi_CpCi * J_CpCi_TCM;
      double constexpr sigma_px = 1.0;
      information_matrix += (1./std::pow(sigma_px,2)) * J_IpCi_TCM.transpose() * J_IpCi_TCM;
    }

    // Get the covariance matrix
    Eigen::Matrix<double,6,6> cov_TCM = information_matrix.inverse();
    cov_TCM.block<3,3>(0,0) *= std::pow(DEG,2);

    // Add the measurement
    marker_ptr->addMeasurement(timestamp_ns, RuM, TCM, cov_TCM);
    detected_markers_ptr->push_back(std::move(marker_ptr));
  }

  #if USE_TEST_BENCH
    cv::imshow("Image", image_with_markers);
    cv::waitKey(1000);
  #endif

  return true;
}

//--------------------------------------------------------------------------------------------------

bool Camera::takePicture(cv::Mat* image_ptr, double timeout)
{
  CHECK_NOTNULL(image_ptr);
  cv::Mat& image = *image_ptr;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!isRunningOnRPi_)
  {
    // Dummy image.
    image = cv::Mat(2,2, CV_8UC1, cv::Scalar(0,0,255));
    return true;
  }
  #ifdef RPI4
    LibcameraOutData frameData;
    bool success = false;
    struct timespec st, ct;
    clock_gettime(CLOCK_MONOTONIC, &st);
    ct = st;

    while (!success && (ct.tv_sec - st.tv_sec + (ct.tv_nsec - st.tv_nsec) / 1e9) < timeout)
    {
      success = camera_.readFrame(&frameData);
      clock_gettime(CLOCK_MONOTONIC, &ct);
    }

    if (success)
    {
      image = cv::Mat(image_height_, image_width_, CV_8UC3, frameData.imageData).clone();
      camera_.returnFrameBuffer(frameData);
    }
  #else
    bool success = camera_handler_.open();
    success &= camera_handler_.grab(),
    camera_handler_.retrieve(image);
    camera_handler_.release();
  #endif

  return success;
}

//--------------------------------------------------------------------------------------------------

Camera::Params Camera::Params::getDefaultParams()
{
  // Set the camera parameters
  camera::Camera::Params params;
  params.name = "camera";
  params.resolution[camera::Camera::Params::WIDTH]  = 1280;
  params.resolution[camera::Camera::Params::HEIGHT] =  960;
  params.intrinsics[camera::Camera::Params::FX]     = 1368.818;
  params.intrinsics[camera::Camera::Params::FY]     = 1358.929;
  params.intrinsics[camera::Camera::Params::CX]     =  542.308;
  params.intrinsics[camera::Camera::Params::CY]     =  476.351;
  params.distortion_model = camera::DistortionModel::Type::NoDistortion;
  params.distortion_coeffs = {0., 0., 0., 0., 0.};
  params.pose = Eigen::Affine3d::Identity();
  return params;
}

//--------------------------------------------------------------------------------------------------

void Camera::configureCamera()
{
  std::lock_guard<std::mutex> lock(mutex_);
  #ifdef RPI4
  isRunningOnRPi_ = !camera_.initCamera(image_width_, image_height_, formats::RGB888, 1, 0);
  if(isRunningOnRPi_)
  {
    ControlList controls_;
    int64_t frame_time = 1000000 / 30; // 30FPS
    controls_.set(controls::FrameDurationLimits, { frame_time, frame_time });
    controls_.set(controls::Brightness, 0.1);
    controls_.set(controls::Contrast, 1.0);
    camera_.set(controls_);
    camera_.startCamera();
  }
  #else
  camera_handler_ = raspicam::RaspiCam_Cv();
  camera_handler_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  camera_handler_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  camera_handler_.set(cv::CAP_PROP_FORMAT, CV_8UC1);
  camera_handler_.set(cv::CAP_PROP_BRIGHTNESS, 75);
  camera_handler_.set(cv::CAP_PROP_CONTRAST, 75);
  isRunningOnRPi_ = camera_handler_.getId().length() > 0;
  #endif
}

//--------------------------------------------------------------------------------------------------

void Camera::normalize(Eigen::Vector2d* point_2d) const
{
  CHECK_NOTNULL(point_2d);
  Eigen::Vector3d point_3d;
  backProject(*point_2d, &point_3d);
  point_2d->x() = point_3d.x()/image_width_;
  point_2d->y() = point_3d.y()/image_height_;
}

//--------------------------------------------------------------------------------------------------

void Camera::backProject(
  Eigen::Vector2d const& point2d,
  Eigen::Vector3d* point3d) const
{
  CHECK_NOTNULL(point3d);
  double const xd = (point2d(0) - intrinsics_.cx) / intrinsics_.fx;
  double const yd = (point2d(1) - intrinsics_.cy) / intrinsics_.fy;
  Eigen::Vector2d point{xd, yd};
  distortion_->undistort(&point);
  *point3d = Eigen::Vector3d{xd, yd, 1.0};
  point3d->normalize();
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
