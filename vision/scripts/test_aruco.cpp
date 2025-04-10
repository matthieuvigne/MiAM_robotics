// Command line : ./test_aruco ../data/test_aruco.jpg

#include <iostream>
#include <cassert>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

// Parameters
bool constexpr show_images = false;

// Functions
Eigen::Matrix3d skew(Eigen::Vector3d const& w)
{
  Eigen::Matrix3d res = Eigen::Matrix3d::Zero();
  res(0,1) = - w.z();
  res(1,0) =   w.z();
  res(0,2) =   w.y();
  res(2,0) = - w.y();
  res(1,2) = - w.x();
  res(2,1) =   w.x();
  return res;
}

// Main function
int main(int argc, char* argv[])
{
  std::cout << "Detection of Aruco tags\n";

  // Load the test image
  std::cout << "Loading the input image..." << std::flush;
  if(argc!=2){
    throw std::invalid_argument("There should be exactly one argument.");
  }
  std::string const& image_path = argv[1];
  cv::Mat const image = cv::imread(image_path,cv::IMREAD_COLOR);
  std::cout << "ok" << std::endl;
  
  // Generate the marker image
  std::cout << "Generate the Marker image..." << std::flush;
  cv::Mat marker_image;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::drawMarker(dictionary, 23, 200, marker_image, 1);
  std::cout << "ok" << std::endl;
  
  // Display the generated marker image
  if(show_images)
  {
    cv::imshow("Generated marker",marker_image);
    cv::waitKey(0);
  }

  // Detect the AruCo markers
  // The order of the detected corners is clockwise
  std::cout << "Detecting the AruCo markers..." << std::flush;
  std::vector<int> detected_marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(image, dictionary, marker_corners, detected_marker_ids, parameters, rejected_candidates);
  std::cout << "ok" << std::endl;
  
  // Show the detected AruCo markers
  if(show_images)
  {
    cv::Mat image_with_markers = image.clone();
    cv::aruco::drawDetectedMarkers(image_with_markers, marker_corners, detected_marker_ids);
    cv::imshow("Detected markers",image_with_markers);
    cv::waitKey(0);
  }

  // Estimate the relative pose of the markers w.r.t. the camera frame
  // The frame attached to the marker is built such as the 3D coordinates of the ordered
  // provided corners are (-markerLength/2, markerLength/2, 0), (markerLength/2, markerLength/2, 0),
  // (markerLength/2, -markerLength/2, 0), (-markerLength/2, -markerLength/2, 0) 
  std::cout << "Estimate the relative pose of the markers w.r.t. the camera frame..."<< std::flush;
  cv::Size const image_size = image.size();
  cv::Mat camera_matrix = cv::Mat::eye(3,3,CV_64FC1);
  camera_matrix.at<double>(0,0) = 650.;
  camera_matrix.at<double>(1,1) = 650.;
  camera_matrix.at<double>(0,2) = image_size.width/2.;
  camera_matrix.at<double>(1,2) = image_size.height/2.;
  cv::Mat const distortion_coeffs = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0)); 
  std::vector<cv::Vec3d> rvecs, tvecs;
  double constexpr marker_length = 0.05;
  cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs);
  std::cout << "ok" << std::endl;
  
  // Draw the estimate axes
  if(show_images)
  {
    cv::Mat image_with_axes = image.clone();
    for (int i = 0; i < rvecs.size(); ++i)
    {
      cv::Vec3d rvec = rvecs[i];
      cv::Vec3d tvec = tvecs[i];
      cv::aruco::drawAxis(image_with_axes, camera_matrix, distortion_coeffs, rvec, tvec, 0.1);
    }
    cv::imshow("Estimated marker axes",image_with_axes);
    cv::waitKey(0);
  }

  // Estimate the covariance matrix of the detected markers by solving a PnP problem
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6d;
  int const num_markers = detected_marker_ids.size();
  for(int marker_idx=0; marker_idx<num_markers; ++marker_idx)
  {
    // Get the detected marker's ID
    int const marker_id = detected_marker_ids[marker_idx];
    std::cout << "Marker ID: " << marker_id << std::endl;

    // Get the marker's estimated relative pose
    cv::Vec3d const& cv_C_r_M = rvecs[marker_idx];
    cv::Vec3d const& cv_C_t_M = tvecs[marker_idx];
    cv::Mat cv_R_CM = cv::Mat::zeros(3,3,CV_64F);
    cv::Rodrigues(cv_C_r_M,cv_R_CM);
    
    // Get the marker's corner image coordinates
    int constexpr num_corners = 4;
    std::vector<cv::Point2f> const& corners = marker_corners[marker_idx];
    if(corners.size() != num_corners) throw "Wrong number of corners!";
    
    // Convert the OpenCV objects to Eigen objects
    Eigen::Matrix3d R_CM, K;
    cv::cv2eigen(cv_R_CM,R_CM);
    cv::cv2eigen(camera_matrix,K);
    Eigen::Vector3d C_t_M;
    cv::cv2eigen(cv_C_t_M,C_t_M);
    Eigen::Affine3d T_CM = Eigen::Affine3d::Identity();
    T_CM.translate(C_t_M).rotate(R_CM);
    
    // Initialize the Fisher Information Matrix
    Eigen::Matrix<double,6,6> I = Eigen::Matrix<double,6,6>::Zero();
    
    // Compute the predictif markers' coordinates
    for(int corner_idx=0; corner_idx<num_corners; ++corner_idx)
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
        default:
          throw "Fucking shit";
      }
      
      // Compute the projection of the marker on the image plan
      Eigen::Vector3d const C_p_Ci = T_CM * M_p_Ci;
      Eigen::Vector3d const I_p_Ci = K * C_p_Ci;
      
      // Update the Fisher information matrix for the marker
      Eigen::Matrix<double,3,6> J = Eigen::Matrix<double,3,6>::Zero();
      J.block<3,3>(0,0) = - skew(R_CM*M_p_Ci);
      J.block<3,3>(0,3).setIdentity();
      J = K * J;
      Eigen::Matrix<double,2,3> Jh = Eigen::Matrix<double,2,3>::Zero();
      Jh.block<2,2>(0,0).setIdentity();
      Jh(0,2) = -I_p_Ci.x()/I_p_Ci.z();
      Jh(1,2) = -I_p_Ci.y()/I_p_Ci.z();
      Jh = (1./I_p_Ci.z()) * Jh;
      Eigen::Matrix<double,2,6> const Jac = Jh * J;
      double constexpr sigma_vis = 0.8;
      I += (1./std::pow(sigma_vis,2)) * Jac.transpose() * Jac;
    }
    
    // Display the covariance matrix
    std::cout << "Covariance matrix:\n" << I.inverse() << std::endl;
  }
  
  return EXIT_SUCCESS;
}
