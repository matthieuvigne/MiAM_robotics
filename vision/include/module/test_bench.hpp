#ifndef MODULE_TEST_BENCH_HPP
#define MODULE_TEST_BENCH_HPP

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

/* Make the testBench a singleton class, with an extern global variable.
 */

namespace module {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class TestBench {

public:
  enum class Mode { PERFECT, NOISY };
  struct Options;

public:
  POINTER_TYPEDEF(TestBench);
  TestBench(Options const& options);

public:

  // Initialization and parameter settings
  static void init(Options const& options);
  void setCameraRotationProcessNoise(double camera_sigma_w);
  void setMarkerRotationMeasurementNoise(double marker_sigma_w);
  void setMarkerPositionMeasurementNoise(double marker_sigma_t);

  // Getters
  inline Eigen::Affine3d const& getTWC(){ CHECK(is_initialized_); return TWC_; }
  inline Eigen::Affine3d const& getTRC(){ CHECK(is_initialized_); return TRC_; }
  inline double getCameraRotationProcessNoise(){ return camera_sigma_w_; };
  inline double getMarkerRotationMeasurementNoise(){ return marker_sigma_w_; }
  inline double getMarkerPositionMeasurementNoise(){ return marker_sigma_t_; }
  inline common::MarkerIdToPose const& getTrueMarkerPoses(){ return markers_; }
  double getCameraRotationTime(double angle_rad);

  // Simulation of the test bench
  //~ void rotateCamera(double wx_rad, double wy_rad, double wz_rad);
  void rotateCamera(double dtheta_rad);
  void rotateCameraToAnglePosition(double angle_rad); 
  void detectMarkers(common::DetectedMarkerList* detected_markers, Mode mode = Mode::NOISY);

public:

  // Board dimensions
  double board_width_  = 3.0;
  double board_height_ = 2.0;

private:

  // Test bench
  static bool is_initialized_;

  // Camera and marker true poses
  Eigen::Affine3d TWC_; ///< Transformation from the world frame to the camera frame
  Eigen::Affine3d TRC_; ///< Transformation from the reference frame to the camera frame
  double camera_angular_speed_ = 10*RAD;

  // Robots and samples true poses
  common::MarkerIdToPose markers_;

  // Process noise and measurement noise parameters
  double camera_sigma_w_;
  double marker_sigma_w_;
  double marker_sigma_t_;

}; // class TestBench

//--------------------------------------------------------------------------------------------------

struct TestBench::Options {

  // Pose
  Eigen::Affine3d TWC;  ///< Camera pose wrt. the world frame
  Eigen::Affine3d TRC;  ///< Camera pose wrt. the reference frame
  Eigen::Affine3d TWM;  ///< Central marker pose wrt. the world frame

  // Noise parameters
  Mode mode = Mode::NOISY;          ///< Simulate uncertainty ?
  double camera_sigma_w = 1.0*RAD;  ///< Camera rotation process noise
  double marker_sigma_w = 1.0*RAD;  ///< Marker measurement orientation stddev
  double marker_sigma_t = 1e-2;     ///< Marker measurement position stddev
  double TWCi_sigma_w = 3.0*RAD;    ///< Camera initial pose sampling orientation stddev
  double TWCi_sigma_t = 1e-2;       ///< Camera initial pose sampling position stddev
  double TRC_sigma_w  = 1.0*RAD;    ///< Reference to camera pose sampling orientation stddev
  double TRC_sigma_t  = 1e-3;       ///< Reference to camera pose sampling position stddev
  double TWM_sigma_w  = 1.0*RAD;    ///< Central marker pose sampling orientation stddev
  double TWM_sigma_t  = 5e-3;       ///< Central marker pose sampling position stddev

  // Functions
  static Options getDefaultOptions();

}; // struct TestBench::Options

//--------------------------------------------------------------------------------------------------
// Global variable declaration
//--------------------------------------------------------------------------------------------------

extern TestBench::UniquePtr test_bench_ptr;

//--------------------------------------------------------------------------------------------------

} // namespace module

//--------------------------------------------------------------------------------------------------
// Global variable declaration
//--------------------------------------------------------------------------------------------------

#define TEST_BENCH_PTR module::test_bench_ptr

//--------------------------------------------------------------------------------------------------

#endif // MODULE_TEST_BENCH
