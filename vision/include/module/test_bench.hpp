#ifndef MODULE_TEST_BENCH_HPP
#define MODULE_TEST_BENCH_HPP

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

namespace module {

namespace test {

//--------------------------------------------------------------------------------------------------
// Variables
//--------------------------------------------------------------------------------------------------

namespace /*Anonymous*/ {

// Test bench
bool is_initialized_ = false;

// Board dimensions
double constexpr board_width_ = 3.0;
double constexpr board_height_ = 2.0;

// Camera and marker true poses
Eigen::Affine3d TWC_; ///< Transformation from the world frame to the camera frame
Eigen::Affine3d TRC_; ///< Transformation from the reference frame to the camera frame

// Robots and samples true poses
common::MarkerIdToPose markers_; // [TODO] << Add the central marker to the list

// Process noise and measurement noise parameters
double camera_sigma_w_ = 1.0*RAD; 
double marker_sigma_w_ = 1.0*RAD;
double marker_sigma_t_ = 1.0*CM;

} // anonymous namespace

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

// Initialization and parameter settings
void initializeTestBench();
inline bool isInitialized(){ return is_initialized_; }
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

// Simulation of the test bench
void rotateCamera(double wx_rad, double wy_rad, double wz_rad);
void detectMarkers(common::DetectedMarkerList* detected_markers);

//--------------------------------------------------------------------------------------------------

} // namespace test

} // namespace module

#endif // MODULE_TEST_BENCH
