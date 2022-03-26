#include <vision/camera.hpp>
#include <vision/distortion_fisheye.hpp>
#include <vision/distortion_null.hpp>
#include <vision/distortion_radtan.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Camera::Camera(
  std::string const& name,
  uint32_t image_width, uint32_t image_height,
  double fx, double fy, double cx, double cy,
  DistortionModel::UniquePtr& distortion,
  Eigen::Affine3d const& pose)
: name_             (name),
  image_width_      (image_width),
  image_height_     (image_height),
  intrinsics_       {fx,fy,cx,cy},
  camera_handler_   (new raspicam::RaspiCam_Cv),
  distortion_       (std::move(distortion)),
  dictionary_       (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
  detector_params_  (cv::aruco::DetectorParameters::create()),
  pose_             (pose)
{
  assert(this->distortion_ != nullptr);
  
  // Configure the camera handler to get the photos
  this->camera_handler_->set(CV_CAP_PROP_FRAME_WIDTH, this->image_width_);
  this->camera_handler_->set(CV_CAP_PROP_FRAME_HEIGHT, this->image_height_);
  this->camera_handler_->set(CV_CAP_PROP_FORMAT, CV_8UC1);
}

//--------------------------------------------------------------------------------------------------

Camera::Camera(CameraParams const& params)
: name_             (params.name),
  pose_             (params.pose),
  camera_handler_   (new raspicam::RaspiCam_Cv),
  dictionary_       (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
  detector_params_  (cv::aruco::DetectorParameters::create())
{
  // Get the image resolution
  this->image_width_ = params.resolution[0];
  this->image_height_ = params.resolution[1];
  
  // Get the intrinsic parameters
  this->intrinsics_.fx = params.intrinsics[0];
  this->intrinsics_.fy = params.intrinsics[1];
  this->intrinsics_.cx = params.intrinsics[2];
  this->intrinsics_.cy = params.intrinsics[3];
  
  // Build the camera distortion model
  switch(params.distortion_model)
  {
    case DistortionModel::Type::NoDistortion:
    {
      this->distortion_ = DistortionModel::UniquePtr(new DistortionNull());
      break;
    }
    
    case DistortionModel::Type::RadTan:
    {
      Eigen::Map<Eigen::VectorXd const> const coeffs(params.distortion_coeffs.data(), 5u);
      this->distortion_ = DistortionModel::UniquePtr(new DistortionRadTan(coeffs));
      break;
    }

    case DistortionModel::Type::Fisheye:
    {
      Eigen::Map<Eigen::VectorXd const> const coeffs(params.distortion_coeffs.data(), 4u);
      this->distortion_ = DistortionModel::UniquePtr(new DistortionFisheye(coeffs));
      break;
    }
    
    default: throw("Unrecognized distortion model.");
  }

  // Configure the camera handler to get the photos
  this->camera_handler_->set(CV_CAP_PROP_FRAME_WIDTH, this->image_width_);
  this->camera_handler_->set(CV_CAP_PROP_FRAME_HEIGHT, this->image_height_);
  this->camera_handler_->set(CV_CAP_PROP_FORMAT, CV_8UC1);
}

//--------------------------------------------------------------------------------------------------

Camera::UniquePtr Camera::buildCameraFromYaml(
  std::string const& camera_name,
  YAML::Node const& camera_node)
{
  // Parse the camera resolution
  std::vector<double> const resolution = camera_node["resolution"].as<std::vector<double>>();
  int32_t const image_width = static_cast<int32_t>(resolution[0]);
  int32_t const image_height = static_cast<int32_t>(resolution[1]);
  
  // Get the intrinsic parameters
  std::vector<double> const intrinsics = camera_node["intrinsics"].as<std::vector<double>>();
  double const fx = intrinsics[0];
  double const fy = intrinsics[1];
  double const cx = intrinsics[2];
  double const cy = intrinsics[3];
  
  // Get the camera distortion model
  YAML::Node const& distortion_node = camera_node["distortion"];
  std::string const model_type = distortion_node["model"].as<std::string>();
  DistortionModel::UniquePtr distortion_ptr = nullptr;
  if(model_type == "null")
  {
    distortion_ptr = DistortionModel::UniquePtr(new DistortionNull());
  }
  else if(model_type == "radial-tangential")
  {
    std::vector<double> distortion_coeffs =
      distortion_node["coeffs"].as<std::vector<double>>();
    Eigen::Map<Eigen::VectorXd> coeffs(distortion_coeffs.data(), 5u);
    distortion_ptr = DistortionModel::UniquePtr(new DistortionRadTan(coeffs));
  }
  else if(model_type == "fisheye")
  {
    std::vector<double> distortion_coeffs =
      distortion_node["coeffs"].as<std::vector<double>>();
    Eigen::Map<Eigen::VectorXd> coeffs(distortion_coeffs.data(), 4u);
    distortion_ptr = DistortionModel::UniquePtr(new DistortionFisheye(coeffs));
  }
  assert(distortion_ptr != nullptr);
  
  // Build and return the camera
  Camera::UniquePtr camera_ptr(new Camera(
    camera_name, image_width, image_height,
    fx, fy, cx, cy, distortion_ptr, Eigen::Affine3d::Identity()));
  return std::move(camera_ptr);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Camera::getCameraMatrix(Eigen::Matrix3d* matrix_ptr) const
{
  assert(matrix_ptr != NULL);
  Eigen::Matrix3d& matrix = (*matrix_ptr);
  matrix.setIdentity();
  matrix(0,0) = this->intrinsics_.fx;
  matrix(1,1) = this->intrinsics_.fy;
  matrix(0,2) = this->intrinsics_.cx;
  matrix(1,2) = this->intrinsics_.cy;
}

//--------------------------------------------------------------------------------------------------

void Camera::getCameraMatrix(cv::Mat* matrix_ptr) const
{
  assert(matrix_ptr != NULL);
  cv::Mat& matrix = (*matrix_ptr);
  matrix = cv::Mat::eye(3, 3, CV_64FC1);
  matrix.at<double>(0,0) = this->intrinsics_.fx;
  matrix.at<double>(1,1) = this->intrinsics_.fy;
  matrix.at<double>(0,2) = this->intrinsics_.cx;
  matrix.at<double>(1,2) = this->intrinsics_.cy;
}

//--------------------------------------------------------------------------------------------------

ProjectionResult Camera::project(
  Eigen::Vector3d const& point_3d,
  Eigen::Vector2d* point_2d_ptr,
  Eigen::Matrix<double,2,3>* out_jacobian) const
{
  // Project the point onto the focal image plane
  assert(point_2d_ptr != NULL);
  assert(point_3d.z() != 0.);
  Eigen::Vector2d& point_2d = *point_2d_ptr;
  point_2d(0) = point_3d.x() / point_3d.z();
  point_2d(1) = point_3d.y() / point_3d.z();

  // Apply distortion
  Eigen::Matrix2d J_Fxd_Fxu;
  if(out_jacobian)
    this->distortion_->distort(&point_2d, &J_Fxd_Fxu);
  else
    this->distortion_->distort(&point_2d, NULL);
  
  // Project onto the image plane
  double const& fx = this->intrinsics_.fx;
  double const& fy = this->intrinsics_.fy;
  double const& cx = this->intrinsics_.cx;
  double const& cy = this->intrinsics_.cy;
  double const xd = point_2d(0);
  double const yd = point_2d(1);
  point_2d(0) = fx*xd + cx;
  point_2d(1) = fy*yd + cy;

  // Check wheter the point is visible on the image or not
  bool is_visible = true;
  is_visible &= (point_2d(0) >= 0. && point_2d(0) <= this->image_width_);
  is_visible &= (point_2d(1) >= 0. && point_2d(1) <= this->image_height_);
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
  DetectedMarkerList* detected_markers_ptr
) const
{
  // Check the inputs
  assert(detected_markers_ptr != NULL);
  
  // Get all the markers on the image
  std::vector<int> detected_marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(image, this->dictionary_, marker_corners, detected_marker_ids,
    this->detector_params_, rejected_candidates);
  
  // Estimate the relative pose of the markers w.r.t. the camera
  std::vector<cv::Vec3d> rvecs, tvecs;
  double constexpr marker_length = 0.05;
  cv::Mat camera_matrix, distortion_coeffs;
  this->distortion_->getDistortionCoeffs(&distortion_coeffs);
  this->getCameraMatrix(&camera_matrix);
  cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, 
    camera_matrix, distortion_coeffs, rvecs, tvecs);
  
  // Compute the covariance matrix for all detected markers
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6d;
  int const num_markers = detected_marker_ids.size();
  for(int marker_idx=0; marker_idx<num_markers; ++marker_idx)
  {
    // Initialize the structure
    DetectedMarker marker;
    marker.marker_id = detected_marker_ids[marker_idx];

    // Get the marker's estimated relative pose
    Eigen::Map<Eigen::Vector3d> C_t_CM((double*) tvecs[marker_idx].val);
    Eigen::Map<Eigen::Vector3d> rvec((double*) rvecs[marker_idx].val);
    double const angle = rvec.norm();
    Eigen::Vector3d const axis = rvec.normalized();
    Eigen::Matrix3d const R_CM = Eigen::AngleAxisd(angle,axis).toRotationMatrix();
    Eigen::Affine3d T_CM = Eigen::Affine3d::Identity();
    T_CM.translate(C_t_CM);
    T_CM.rotate(R_CM);
    marker.T_CM = T_CM;
    
    // Get the marker's corner image coordinates
    int constexpr num_corners = 4;
    std::vector<cv::Point2f> const& corners = marker_corners[marker_idx];
    if(corners.size() != num_corners) throw "Wrong number of corners!";
    
    // Initialize the Fisher Information Matrix
    Eigen::Matrix<double,6,6> information_matrix = Eigen::Matrix<double,6,6>::Zero();
    
    // Compute the predictif markers' coordinates
    for(int corner_idx=1; corner_idx<num_corners; ++corner_idx)
    {
      // Get the corner's coordinates w.r.t. the marker's frame
      Eigen::Vector3d M_p_Ci;
      switch(corner_idx)
      {
        case 0:
          M_p_Ci = Eigen::Vector3d(-marker_length/2.,marker_length/2.,0);
          break;
        case 1:
          M_p_Ci = Eigen::Vector3d(marker_length/2.,marker_length/2.,0);
          break;
        case 2:
          M_p_Ci = Eigen::Vector3d(marker_length/2.,-marker_length/2.,0);
          break;
        case 3:
          M_p_Ci = Eigen::Vector3d(-marker_length/2.,-marker_length/2.,0);
          break;
      }
      
      // Project the 3d corner into the camera's frame
      Eigen::Vector3d const C_p_Ci = T_CM * M_p_Ci;
      Eigen::Matrix<double,3,6> J_CpCi_TCM;
      J_CpCi_TCM.block<3,3>(0,0) = - Camera::skew(R_CM*M_p_Ci);
      J_CpCi_TCM.block<3,3>(0,3).setIdentity();

      // Project the point onto the image plane
      Eigen::Vector2d I_p_Ci;
      Eigen::Matrix<double,2,3> J_IpCi_CpCi;
      this->project(C_p_Ci, &I_p_Ci, &J_IpCi_CpCi);

      // Update the Fisher Information Matrix
      Eigen::Matrix<double,2,6> const J_IpCi_TCM = J_IpCi_CpCi * J_CpCi_TCM;
      double constexpr sigma_vis = 0.8;
      information_matrix += (1./std::pow(sigma_vis,2)) * J_IpCi_TCM.transpose() * J_IpCi_TCM;
    }
    
    // Assign the covariance matrix
    marker.cov_T_CM = information_matrix.inverse();
    detected_markers_ptr->push_back(marker);
  }
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d Camera::skew(Eigen::Vector3d const& v)
{
  Eigen::Matrix3d res;
  res <<     0., -v.z(),  v.y(),
          v.z(),     0., -v.x(),
         -v.y(),  v.x(),     0.;
  return res;
}

//--------------------------------------------------------------------------------------------------

bool Camera::takePicture(cv::Mat* image) const
{
  bool success = this->camera_handler_->open();
  success &= this->camera_handler_->grab(),
  this->camera_handler_->retrieve(*image);
  this->camera_handler_->release();
  return success;
}

//--------------------------------------------------------------------------------------------------

#if 0
//~ // As options, pass the message queue and its associated mutex from the board.
//~ void Camera::cameraThread()
//~ {
  //~ std::cout << "Launched thread for camera " << this->name_ << "." << std::endl;

  //~ while(true)
  //~ {
    //~ // Check if there are specific request
    //~ // Otherwise, sweep
  //~ }

  //~ while(true)
  //~ {
    //~ // Wait condition (unbusy waiting)
    //~ std::unique_lock<std::mutex> camera_locker(this->thread_mtx_);
    //~ this->thread_con_.wait(camera_locker,
      //~ [&](){ return this->thread_image_ != nullptr or this->abort_thread_; });
    //~ if(this->abort_thread_) break;
    //~ vision_mgs::Image::UniquePtr image_msg = std::move(this->thread_image_);
    //~ assert(this->thread_image_ == nullptr);
    //~ this->thread_mtx_.unlock();

    //~ // Process the image
    //~ // Specific processing for fisheye images : TODO
    //~ std::cout << "Processing the new image" << std::endl;
    //~ DetectedMarkerList detected_markers;
    //~ cv::Mat& new_image = *(image_msg->image_ptr);
    //~ this->detectMarkers(new_image, &detected_markers);

    //~ // Sort the markers (according to their numbers)
    //~ // Marker 42 for center tag on the table
    //~ // Marker 47 for the treasure face of red samples
    //~ // Marker 13 for the treasure face of blue samples
    //~ // Marker 36 for the treasure face of geen samples
    //~ // Marker 17 for the rock face of all samples
    //~ // Team purple receives tags between 1 and 5
    //~ // Team yellow receives tags between 6 and 10
    //~ // Tags from 11 to 50 are reserved for the playing area
    //~ // Tags from 51 to 70 are reserved for team purple
    //~ // Tags from 71 to 90 are reserved for team yellow
    //~ // Tags are 4x4 ArUco tags, 7cm wide, and 10cm for the border
    //~ // All robots will have different markers and it will not be possible to choose them.
    //~ // -> Such markers are laid on the top of the robot, above the eventual beacon.
    //~ // Possible to make beacons with tags (dimensions 510mm high, 100m side) and use the tags we want.
    
    //~ // Build the message to send to the robot and add it to the queue
    //~ // TODO
  //~ }
  
  //~ // Shut the thread down
  //~ std::cout << "Shutting down camera thread." << std::endl;
//~ }
#endif

//--------------------------------------------------------------------------------------------------

} // namespace vision
