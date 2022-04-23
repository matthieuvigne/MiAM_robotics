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

} // anonymous namespace

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

// Actions
void initializeTestBench();
void rotateCamera(double wx_rad, double wy_rad, double wz_rad);
void detectMarkers(common::DetectedMarkerList* detected_markers);

// Getters
Eigen::Affine3d const& getTWC(){ CHECK(is_initialized_); return TWC_; }
Eigen::Affine3d const& getTRC(){ CHECK(is_initialized_); return TRC_; }

//--------------------------------------------------------------------------------------------------

} // namespace test

} // namespace module

#endif // MODULE_TEST_BENCH
