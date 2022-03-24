#include <iostream>
#include <cassert>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

/* Objective : generate a video of a fixed camera with distorsion coeffs
 * and moving AruCo tags.
 * */

// Main routine
int main(int argc, char* argv[])
{
  std::cout << "Script: create AruCo video" << std::endl;

  // Load the game area
  std::string const area_path = "/home/rodolphe/Programming/vision/data/area.png";
  cv::Mat area_img = cv::imread(area_path,cv::IMREAD_COLOR);
  cv::transpose(area_img,area_img);
  cv::imshow("Game area",area_img);
  cv::waitKey(0);
  
  // Get the game area matrix
  int constexpr height = 2.0;
  int constexpr width = 3.0;
  Eigen::Matrix3d Kg = Eigen::Matrix3d::Identity();
  Kg(0,0) = area_img.cols/width;
  Kg(1,1) = area_img.rows/height;
  Kg(0,2) = area_img.cols/2.;
  Kg(1,2) = area_img.rows/2.;
  
  // Get the camera matrix
  cv::Size img_size(752,480);
  Eigen::Matrix3d Kc = Eigen::Matrix3d::Identity();
  Kc(0,0) = 458; //img_size.width/width;
  Kc(1,1) = 457; //img_size.width/width; //img_size.height/height;
  Kc(0,2) = img_size.width/2.;
  Kc(1,2) = img_size.height/2.;
  
  // Generate the tags
  int constexpr num_markers = 10;
  int constexpr marker_size_px = 200;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<cv::Mat> markers(num_markers,cv::Mat());
  for(int marker_idx=0; marker_idx<num_markers; marker_idx++)
    cv::aruco::drawMarker(dictionary, marker_idx, marker_size_px, markers[marker_idx], 1);
  
  // Define the pose of the game area
  Eigen::Affine3d const T_WC0 =
    Eigen::Translation3d(Eigen::Vector3d(height/2.,width/2.,0))
    * Eigen::AngleAxisd(M_PI_2,Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX());
  
  // Define the marker camera matrix
  double constexpr marker_size_m = 7e-2;
  Eigen::Matrix3d Km = Eigen::Matrix3d::Identity();
  Km(0,0) = marker_size_px/marker_size_m;
  Km(1,1) = marker_size_px/marker_size_m;
  Km(0,2) = marker_size_px/2.;
  Km(1,2) = marker_size_px/2.;
  
  for(int i=0; i<10; ++i)
  {  
    // Compute the camera pose
    Eigen::Affine3d const T_WC1 =
      Eigen::Translation3d(Eigen::Vector3d(0,1.425,1.))
      * Eigen::AngleAxisd(M_PI_2,Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(-M_PI_2,Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(-(1.+i/10.)*M_PI_4,Eigen::Vector3d::UnitX());
    Eigen::Affine3d const T_C0C1 = T_WC0.inverse() * T_WC1;
    
    // Compute the 2D homography
    Eigen::Matrix3d const R_C0C1 = T_C0C1.rotation();
    Eigen::Vector3d const C0_t_C1 = T_C0C1.translation();
    Eigen::Matrix3d const H = Kc * R_C0C1.transpose()
      * (Kg.inverse() - C0_t_C1 * Eigen::Vector3d::UnitZ().transpose());
    cv::Mat Hcv;
    cv::eigen2cv(H,Hcv);
    
    // Apply the 2D homography to the game area
    cv::Mat dst(img_size,CV_8UC3);
    cv::warpPerspective(area_img,dst,Hcv,img_size);
    cv::imshow("Game area",dst);
    cv::waitKey(0);
  }

  return EXIT_SUCCESS;
}
