#ifndef MODULE_TEST_BENCH_HPP
#define MODULE_TEST_BENCH_HPP

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

namespace module {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class TestBench {

public:
  DISALLOW_EVIL_CONSTRUCTORS(TestBench);

public:

  // Initialization and parameter settings
  static void initializeTestBench();
  inline static bool isInitialized(){ return is_initialized_; }
  static void setCameraRotationProcessNoise(double camera_sigma_w);
  static void setMarkerRotationMeasurementNoise(double marker_sigma_w);
  static void setMarkerPositionMeasurementNoise(double marker_sigma_t);

  // Getters
  inline static Eigen::Affine3d const& getTWC(){ CHECK(is_initialized_); return TWC_; }
  inline static Eigen::Affine3d const& getTRC(){ CHECK(is_initialized_); return TRC_; }
  inline static double getCameraRotationProcessNoise(){ return camera_sigma_w_; };
  inline static double getMarkerRotationMeasurementNoise(){ return marker_sigma_w_; }
  inline static double getMarkerPositionMeasurementNoise(){ return marker_sigma_t_; }
  inline static common::MarkerIdToPose const& getTrueMarkerPoses(){ return markers_; }
  static double getCameraRotationTime(double angle_rad);

  // Simulation of the test bench
  static void rotateCamera(double wx_rad, double wy_rad, double wz_rad);
  static void detectMarkers(common::DetectedMarkerList* detected_markers);

public:

  // Board dimensions
  static double board_width_;
  static double board_height_;

private:

  // Test bench
  static bool is_initialized_;

  // Camera and marker true poses
  static Eigen::Affine3d TWC_; ///< Transformation from the world frame to the camera frame
  static Eigen::Affine3d TRC_; ///< Transformation from the reference frame to the camera frame
  static double camera_angular_speed_;

  // Robots and samples true poses
  static common::MarkerIdToPose markers_; // [TODO] << Add the central marker to the list

  // Process noise and measurement noise parameters
  static double camera_sigma_w_; 
  static double marker_sigma_w_;
  static double marker_sigma_t_;

}; // class TestBench

//--------------------------------------------------------------------------------------------------

} // namespace module

#endif // MODULE_TEST_BENCH
