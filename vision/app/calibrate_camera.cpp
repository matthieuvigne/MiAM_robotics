#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

#include <common/macros.hpp>

//--------------------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------------------

struct ChessBoard {
  int height  = 5;      // Number of squares
  int width   = 5;      // Number of squares
  double size = 28e-3;  // Size of a square in millimeters
}; // struct ChessBoard
ChessBoard CHESSBOARD;

//--------------------------------------------------------------------------------------------------
// Main routine
//--------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  // Command line parser
  std::string const keys =
    "{image_folder      | /home/rodolphe/Pictures/pictures/serie4  | Calibration image folder   }"
    "{output_filename   |                                          | Output file name           }"
    "{display_images    | false                                    | Set true to dispkay images }";
  cv::CommandLineParser parser(argc, argv, keys);
  if(!parser.check())
  {
    LOG("Invalid command line arguments. Aborting the script.");
    return EXIT_FAILURE;
  }
  parser.about("Camera calibration script");
  std::string const image_folder    = parser.get<std::string> ("image_folder");
  std::string const output_filename = parser.get<std::string> ("output_filename");
  bool const display_images         = parser.get<bool>        ("display_images");

  // Get all the images in the image folders
  LOG("Get all the images for calibration");
  std::vector<cv::String> images;
  std::string const image_path = image_folder + "/*.jpg";
  cv::glob(image_path, images);
  size_t const num_images = images.size();
  LOG("Found " << num_images << " images.");

  // Defining the chessboard model
  std::vector<cv::Point3f> chessboard_points;
  for(int i=0; i<CHESSBOARD.width; i++)
  {
    for(int j=0; j<CHESSBOARD.height; j++)
      chessboard_points.push_back(
        cv::Point3f(i*CHESSBOARD.size,j*CHESSBOARD.size,0));
  }
  
  // Loop over all the images and detect the chessboard
  LOG("Loop over all the images in the directory and detect the chessboard");
  cv::Size image_size;
  std::vector<std::vector<cv::Point2f> > points_2d;
  std::vector<std::vector<cv::Point3f> > points_3d;
  for(size_t image_idx=0u; image_idx<50/*num_images*/; image_idx++)
  {
    // Load the image and display it
    std::string const& image_path = images[image_idx];
    cv::Mat const original_frame = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    cv::Mat resized_frame;
    cv::resize(original_frame, resized_frame, cv::Size(), 0.25, 0.25);
    image_size = cv::Size(resized_frame.rows, resized_frame.cols);
    if(display_images)
    {
      cv::imshow("image",resized_frame);
      cv::waitKey(250);
    }
    
    // Looking for the corners
    std::vector<cv::Point2f> corner_pts;
    cv::Size const chessboard_size(CHESSBOARD.width, CHESSBOARD.height);
    bool const success = cv::findChessboardCorners(resized_frame, chessboard_size, corner_pts);
    LOG("Image " << image_path << " -> " << (success ? "success" : "failed"));
    
    // If success, refine pixel coordinates and show the detected chessboard
    if(success)
    {
      // Refine the pixel coordinates of the corners
      cv::Size const window_size(11,11);
      cv::Size const zero_zone(-1,-1);
      cv::TermCriteria const criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);
      cv::cornerSubPix(resized_frame, corner_pts, window_size, zero_zone, criteria);
      
      // Display the detected corner points on the chessboard
      if(display_images)
      {
        cv::drawChessboardCorners(resized_frame, chessboard_size, corner_pts, success);
        cv::imshow("image", resized_frame);
        cv::waitKey(250);
      }
      
      // Save the results
      points_3d.push_back(chessboard_points);
      points_2d.push_back(corner_pts);
    }
  }
  cv::destroyAllWindows();
  LOG("Detected the chessboard on " << points_2d.size() << "/" << num_images << " images");
  
  // Calibrate the camera
  cv::Mat K;  // Camera intrinsic matrix
  cv::Mat d;  // Distortion coefficients
  cv::Mat R;  // Camera rotation matrix
  cv::Mat t;  // Camera translation vector
  cv::calibrateCamera(points_3d, points_2d, image_size, K, d, R, t);

  // Display and write the results
  LOG("cameraMatrix : "         << K);
  LOG("distCoeffs : "           << d);
  LOG("Rotation vector : "      << R);
  LOG("Translation vector : "   << t);
  
  //~ std::string const filepath = "test.
  if(!output_filename.empty())
  {
    cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);
    fs << "K" << K;
    fs << "d" << d;
    fs << "R" << R;
    fs << "T" << t;
    fs.release();
  }

  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
