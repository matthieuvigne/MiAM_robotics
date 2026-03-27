#include "TagDetector.h"

TagDetector::TagDetector(int width, int height, double fx, double fy, double cx, double cy)
: width_(width), height_(height),
  fx_(NAN), fy_(NAN),
  cx_(width/2.), cy_(height/2.)
{
  // Check the image resolution
  assert( (width_>0) );
  assert( (height_>0) );

  // Initialize the ARUCO detector parameters
  cv_dico_ptr_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  cv_params_ptr_ = cv::aruco::DetectorParameters::create();

  // Set focal lenghts
  assert( (fx>0.) && (fy>0.) );
  fx_ = fx;
  fy_ = fy;

  // Set optical center coordinates
  assert( (cx>0.) && (cy>0.) );
  cx_ = cx;
  cy_ = cy;
  return;
}

cv::Mat TagDetector::get_camera_matrix() const
{
  cv::Mat K = cv::Mat::eye(3,3,CV_64FC1);
  K.at<double>(0,0) = fx_;
  K.at<double>(1,1) = fy_;
  K.at<double>(0,2) = cx_;
  K.at<double>(1,2) = cy_;
  return K;
}

int TagDetector::detect_markers(
  cv::Mat const& img,
  std::vector<Eigen::Affine3d>* TRM_ptr) const
{
  // Detect the markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::aruco::detectMarkers(img, cv_dico_ptr_, marker_corners, marker_ids,
    cv_params_ptr_, rejected_candidates);

  // Remove all detected markers other than ID47
  assert( marker_ids.size() == marker_corners.size() );
  std::vector<int>::iterator marker_it = marker_ids.begin();
  std::vector<std::vector<cv::Point2f>>::iterator corner_it = marker_corners.begin();
  while( marker_it != marker_ids.end() )
  {
    if(*marker_it != 47u)
    {
      marker_it = marker_ids.erase(marker_it);
      corner_it = marker_corners.erase(corner_it);
    } else {
      marker_it += 1;
      corner_it += 1;
    }
  }

  // Get the number of detected markers
  int const num_markers = static_cast<int>(marker_ids.size());
  if(num_markers == 0) return 0;

  // Estimate the pose of each marker
  std::vector<cv::Vec3d> rvecs, tvecs;
  double constexpr marker_length = 5e-2;
  cv::Mat const K = this->get_camera_matrix();
  cv::Mat dist_coeffs = cv::Mat(4,1,CV_64FC1, cv::Scalar::all(0));
  cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, K, dist_coeffs, rvecs, tvecs);
  assert( (rvecs.size()==num_markers) && (tvecs.size()==num_markers) );

  // Return the pose of each marker in the robot's reference frame
  assert( TRM_ptr != NULL );
  std::vector<Eigen::Affine3d>& TRM = *TRM_ptr;
  TRM.clear();
  TRM.resize(num_markers);
  for(int marker_idx=0; marker_idx<num_markers; marker_idx+=1)
  {
    cv::Vec3d const& rvec = rvecs[marker_idx];
    double const angle_rad = cv::norm(rvec);
    Eigen::Vector3d const axis = Eigen::Vector3d(rvec[0],rvec[1],rvec[2]).normalized();
    Eigen::AngleAxisd const rCM(angle_rad,axis);
    Eigen::Map<Eigen::Vector3d const> tCM(tvecs[marker_idx].val);
    Eigen::Affine3d TCM(Eigen::Translation3d(tCM)*rCM);
    TRM[marker_idx] = TRC_ * TCM;
  }
  return num_markers;
}

MarkerList TagDetector::find_markers(cv::Mat const& img)
{
  assert( img.cols == width_ );
  assert( img.rows == height_ );
  std::vector<Eigen::Affine3d> TRMs;
  int const num_markers = detect_markers(img, &TRMs);

  // Convert into marker polar coordinates
  MarkerList markers;
  markers.resize(num_markers);
  for(int marker_idx=0; marker_idx<num_markers; marker_idx+=1)
  {
    Eigen::Vector3d const pRM = TRMs[marker_idx].translation();
    double const radius = pRM.norm();
    double const theta_rad = std::atan2(pRM.y(),pRM.x());
    markers[marker_idx] = Marker{radius,theta_rad,pRM.z()};
  }

  return markers;
}
