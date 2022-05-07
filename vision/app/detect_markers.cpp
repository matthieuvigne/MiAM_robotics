#include <camera/camera.hpp>
#include <common/logger.hpp>

int main(int argc, char* argv[])
{
  // Initialize the console logger
  common::ConsoleLogger::init();
  
  // Load the test image
  if(argc!=2) throw std::invalid_argument("There should be exactly one argument.");
  std::string const& image_path = argv[1];
  cv::Mat const image = cv::imread(image_path,cv::IMREAD_COLOR);
  
  // Show the image
  cv::imshow("Fenêtre",image);
  cv::waitKey(0);

  // Detect the AruCo markers
  cv::Mat marker_image;
  std::vector<int> detected_marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::detectMarkers(image, dictionary, marker_corners, detected_marker_ids, parameters, rejected_candidates);

  // Draw all the detected markers
  int const num_detected_markers = static_cast<int>(detected_marker_ids.size());
  for(int marker_idx=0; marker_idx<num_detected_markers; marker_idx++)
  {
    // Display the corner coordinates
    int const marker_id = detected_marker_ids[marker_idx];
    std::vector<cv::Point2f> const& corners = marker_corners[marker_idx];
    std::cout << "Marker with id " << marker_id << ":\n";
    for(int corner_idx=0; corner_idx<4; corner_idx++)
    {
      cv::Point2f const& point = corners[corner_idx];
      std::cout << "Corner n°" << corner_idx+1 << ": (" << point.x << ", " << point.y << ")\n";
    }
    std::cout <<std::endl;
  }

  // Draw the detected markers
  cv::Mat image_with_markers = image.clone();
  cv::aruco::drawDetectedMarkers(image_with_markers, marker_corners, detected_marker_ids);
  cv::imshow("Fenêtre",image_with_markers);
  cv::waitKey(0);


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

  cv::Mat image_with_axes = image.clone();
  for (int i = 0; i < rvecs.size(); ++i)
  {
    cv::Vec3d rvec = rvecs[i];
    cv::Vec3d tvec = tvecs[i];
    cv::drawFrameAxes(image_with_axes, camera_matrix, distortion_coeffs, rvec, tvec, 0.1);
  }
  cv::imshow("Estimated marker axes",image_with_axes);
  cv::waitKey(0);

  return EXIT_SUCCESS;
}
