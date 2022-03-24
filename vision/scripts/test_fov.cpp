#include <cassert>
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

// Beta = 60° is a good value
int main(int argc, char* argv[])
{
  // Get the rotation angle
  double alpha_deg = 10.;
  int const alpha_step = 10;
  double const alpha_deg_max = 40.;
  while(alpha_deg<=alpha_deg_max)
  {
    // Get the alpha angle
    double const alpha_rad = alpha_deg*M_PI/180.;
    std::cout << "\nAlpha = " << alpha_deg << "°" << std::endl;

    // Board dimensions
    int const width = 3.0;
    int const height = 2.0;
    
    // Declare the points of interest
    int const num_points = 10;
    Eigen::Matrix3Xd B_points(3,num_points);
    // Faraway points
    B_points.col(0) = Eigen::Vector3d(0,0,0);
    B_points.col(1) = Eigen::Vector3d(width,0,0);
    // Close points
    B_points.col(2) = Eigen::Vector3d(    0, height, 0);
    B_points.col(3) = Eigen::Vector3d(width, height, 0);
    // Middle points
    B_points.col(4) = Eigen::Vector3d(width/2.,0,0);
    B_points.col(5) = Eigen::Vector3d(width/2.,height,0);
    // First third points
    B_points.col(6) = Eigen::Vector3d(width/3.,0,0);
    B_points.col(7) = Eigen::Vector3d(width/3.,height,0);
    // First third points
    B_points.col(8) = Eigen::Vector3d(2*width/3.,0,0);
    B_points.col(9) = Eigen::Vector3d(2*width/3.,height,0);
    
    // Declare the pose of the cameras
    double const beta_deg = (argc==2) ? std::atof(argv[1]) : 0.;
    double const beta_rad = beta_deg*M_PI/180.;
    std::vector<Eigen::Affine3d> T_BC(1);
    Eigen::Affine3d const std_T_BC =
      Eigen::Translation3d(width/2.,height,0.570)
      * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(-M_PI_2,Eigen::Vector3d::UnitX());
    T_BC[0] = std_T_BC
      * Eigen::AngleAxisd(-alpha_rad,Eigen::Vector3d::UnitX());
    //~ T_BC[1] = std_T_BC
      //~ * Eigen::AngleAxisd(beta_rad, Eigen::Vector3d::UnitY())
      //~ * Eigen::AngleAxisd(-alpha_rad,Eigen::Vector3d::UnitX());
    //~ T_BC[2] = std_T_BC
      //~ * Eigen::AngleAxisd(-beta_rad, Eigen::Vector3d::UnitY())
      //~ * Eigen::AngleAxisd(-alpha_rad,Eigen::Vector3d::UnitX());
    int const num_cameras = static_cast<int>(T_BC.size());

    // Print the pose of the cameras
    for(int cam_idx=0; cam_idx<num_cameras; ++cam_idx)
      std::cout << "Camera " << cam_idx << ": " << std::endl << T_BC[cam_idx].matrix() << std::endl;

    // Iterate over the cameras
    for(int cam_idx=0; cam_idx<num_cameras; ++cam_idx)
    {
      // Display the camera
      std::cout << "Camera " << cam_idx << ":" << std::endl;

      // Convert the points into the camera's frame
      Eigen::Affine3d const T_CB = T_BC[cam_idx].inverse();
      Eigen::Matrix3Xd C_points = B_points;
      for(int point_idx=0; point_idx<num_points; ++point_idx)
        C_points.col(point_idx) = T_CB * B_points.col(point_idx);
      
      // Test points are lying in the camera's fov
      for(int point_idx=0; point_idx<num_points; ++point_idx)
      {
        // Get the points
        Eigen::Vector3d const B_p = B_points.col(point_idx);
        Eigen::Vector3d const C_p = C_points.col(point_idx);
        
        // Get the ray and test the angular opening
        Eigen::Vector3d C_u = C_points.col(point_idx).normalized();
        double const angle = std::acos(C_u.dot(Eigen::Vector3d::UnitZ()))*180./M_PI;
        
        // Display the final message
        std::cout << "-> angle(" << point_idx << ") = " << angle
          << " -> " << ((angle<110.) ? "\033[1;32m visible \033[0m" : "\033[1;31m not visible \033[0m")
          << std::endl;
      }
    }
  
    // Prepare next iteration
    alpha_deg += alpha_step;
  }

  return EXIT_SUCCESS;
}
