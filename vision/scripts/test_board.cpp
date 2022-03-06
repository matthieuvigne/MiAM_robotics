#include <iostream>
#include <cassert>
#include <vision/module.hpp>

// Main routine
int main(int argc, char* argv[])
{
  // Build the vision module from parameters file
  std::string const filename = "/home/rodolphe/Programming/vision/data/params.yaml";
  vision::Module module(filename);


  //~ // Get the board image
  //~ std::cout << "Get the board image..." << std::flush;
  //~ std::string const board_filename = "/home/rodolphe/Programming/vision/data/area.png";
  //~ cv::Mat board_img = cv::imread(board_filename);
  //~ cv::transpose(board_img,board_img);
  //~ cv::flip(board_img,board_img,0);
  //~ std::cout << "ok" << std::endl;
  
  //~ // Display the board image
  //~ cv::imshow("Board image",board_img);
  //~ cv::waitKey(0);
  //~ cv::destroyWindow("Board image");
  
  //~ // Build the board game area
  //~ std::cout << "Build the board game area..." << std::flush;
  //~ double constexpr width = 3.0;
  //~ double constexpr height = 2.0;
  //~ vision::Board board(width, height, board_img);
  //~ std::cout << "ok" << std::endl;

  //~ // Add the cameras to the board
  //~ std::string const cam_name = "cam0";
  //~ cv::Size const img_size(752,480);
  //~ double const fx = 300.;
  //~ double const fy = 300.;
  //~ double const cx = img_size.width/2.;
  //~ double const cy = img_size.height/2.;
  //~ Eigen::Affine3d const T_board_cam =
    //~ Eigen::Translation3d(Eigen::Vector3d(0.0,width/2.0,1.0))
    //~ * Eigen::AngleAxisd(0.9*M_PI,Eigen::Vector3d::UnitY())
    //~ * Eigen::AngleAxisd(-M_PI_2,Eigen::Vector3d::UnitZ());
  //~ size_t const camera_idx = board.addNewCamera(
    //~ cam_name,                                               // Camera name
    //~ img_size.width, img_size.height,                        // Image size
    //~ fx, fy, cx, cy,                                         // Camera intrinsics
    //~ vision::DistortionModel::Type::RadTan,                  // Distortion model
    //~ {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05},  // Distortion coefficients
    //~ T_board_cam);                                           // Board to camera transformation

  //~ // Display the board
  //~ std::cout << board.print() << std::endl;

  //~ // Get the view of the board from the camera
  //~ cv::Mat const view_img = board.getCameraViewOfTheBoard(camera_idx);
  //~ cv::imshow("Camera view", view_img);
  //~ cv::waitKey(0);
  //~ cv::destroyWindow("Camera view");

  //~ // Check detection of markers displayed on the board
  //~ // TODO

  return EXIT_SUCCESS;
}
