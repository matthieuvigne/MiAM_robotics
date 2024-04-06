#include <iostream>
#include <cassert>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main(int argc, char const *argv[])
{
  // Check if camera is working
  cv::VideoCapture camera("/dev/video2");
  if(!camera.isOpened())
  {
    std::cerr << "ERROR: Could not open camera." << std::endl;
    return EXIT_FAILURE;
  }

  // Create ARUCO dictionnary
  cv::Ptr<cv::aruco::Dictionary> dictionary = 
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  cv::Ptr<cv::aruco::DetectorParameters> detector_params =
    cv::aruco::DetectorParameters::create();

  // Set null distortion coefficients
  cv::Mat distortion_coeffs = cv::Mat(4,1,CV_64FC1, cv::Scalar::all(0));

  // Detect markers in the camera
  while(true)
  {
    // Get image
    cv::Mat image;
    camera >> image;

    // Create the camera matrix
    uint32_t image_width = image.cols;
    uint32_t image_height = image.rows;
    cv::Mat camera_matrix = cv::Mat::eye(3,3,CV_64FC1);
    camera_matrix.at<double>(0,0) = 542;
    camera_matrix.at<double>(1,1) = 476;
    camera_matrix.at<double>(0,2) = double(image_width)/2.0;
    camera_matrix.at<double>(1,2) = double(image_height)/2.0;

    // Detect all the markers
    std::vector<int> detected_marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, detected_marker_ids,
      detector_params, rejected_candidates);

    // Compute the angle with the detected marker
    double angle_deg = std::nan("");
    cv::Mat image_markers = image.clone();
    size_t const num_detected_markers = detected_marker_ids.size();
    for(size_t marker_idx=0; marker_idx<num_detected_markers; marker_idx+=1)
    {
      // Skip if not the right marker
      if(detected_marker_ids[marker_idx] != 47u)
        continue;
      
      // Estimate and draw the marker's pose
      std::vector<cv::Vec3d> rvecs, tvecs;
      double const marker_length = 5e-2; // [m]
      std::vector<std::vector<cv::Point2f>> markers_corners{marker_corners[marker_idx]};
      cv::aruco::estimatePoseSingleMarkers(markers_corners, marker_length, camera_matrix,
        distortion_coeffs, rvecs, tvecs);
      cv::drawFrameAxes(image_markers, camera_matrix, distortion_coeffs, rvecs[0], tvecs[0], 5e-2);
      
      // Get the rotation matrix
      Eigen::Map<Eigen::Vector3d> const rvec((double*) rvecs[0].val);
      double const angle = rvec.norm();
      Eigen::Vector3d const axis = rvec.normalized();
      Eigen::Matrix3d const RCM(Eigen::AngleAxisd(angle,axis));
    
      // Project the camera axis vector into the marker's plane
      double const rad2deg = 180. / M_PI;
      std::cout << "RCM\n" << RCM << std::endl;
      Eigen::Vector3d C_u_Mref = RCM.col(1);
      Eigen::Vector3d C_u_Cz = Eigen::Vector3d::UnitZ();
      C_u_Mref -= (C_u_Mref.dot(C_u_Cz))*C_u_Cz;
      C_u_Mref /= C_u_Mref.norm();
      Eigen::Vector3d C_u_Cx = Eigen::Vector3d::UnitX();
      angle_deg = std::acos(C_u_Cx.dot(C_u_Mref));
      std::cout << "-> marker " << marker_idx+1 << ": " << angle_deg*rad2deg << "°" << std::endl;
    }
   
    // Display the image (ESC to break)
    cv::aruco::drawDetectedMarkers(image_markers, marker_corners, detected_marker_ids);
    cv::imshow("Image", image_markers);
    if(cv::waitKey(100)==27) break;
  }
    
  return EXIT_SUCCESS;
}
