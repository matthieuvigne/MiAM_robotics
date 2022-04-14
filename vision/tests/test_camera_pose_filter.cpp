#include <camera/camera_pose_filter.hpp>
#include <common/maths.hpp>

//--------------------------------------------------------------------------------------------------
// Main routine
//--------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  // Create the filter
  Eigen::Affine3d const T_WM = Eigen::Affine3d::Identity();
  Eigen::Affine3d const T_RC = Eigen::Affine3d::Identity();
  Eigen::Affine3d const T_WC = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,6> const cov_T_RC = Eigen::Matrix<double,6,6>::Identity();
  camera::CameraPoseFilter filter(T_WM, T_RC, cov_T_RC);
  
  // Generate the measurements
  int constexpr max_idx = 20;
  for(int idx=0; idx<max_idx; idx++)
  {
    // Move the camera and predict
    // [TODO]
    
    // Update with measurements
    // [TODO]
  }
  
  return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------------------------------
