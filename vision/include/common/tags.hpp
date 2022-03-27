#ifndef COMMON_TAGS_HPP
#define COMMON_TAGS_HPP

#include <map>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <common/time.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Structure/Enum declarations
//--------------------------------------------------------------------------------------------------

typedef uint8_t MarkerId;
enum class MarkerFamily {
  UNKNOWN,
  CENTRAL_MARKER,
  ROCK_SAMPLE,
  TREASURE_RED_SAMPLE,
  TREASURE_GREEN_SAMPLE,
  TREASURE_BLUE_SAMPLE,
  PURPLE_TEAM_ROBOT,
  YELLOW_TEAM_ROBOT,
  PURPLE_TEAM_MARKER,
  YELLOW_TEAM_MARKER
}; // enum class MarkerFamily

MarkerFamily getMarkerFamily(MarkerId id);

struct DetectedMarker {
  MarkerId marker_id;
  Eigen::Affine3d T_CM;
  Eigen::Matrix<double,6,6> cov_T_CM;
}; // DetectedMarker
typedef std::vector<DetectedMarker> DetectedMarkerList;

struct MarkerEstimate {
  MarkerId id = 0;
  MarkerFamily family = MarkerFamily::UNKNOWN;
  int64_t timestamp_ns = 0u;
  Eigen::Affine3d T_WM = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,6> cov_T_WM = Eigen::Matrix<double,6,6>::Identity();
}; // class MarkerEstimate
typedef std::vector<MarkerEstimate> MarkerEstimateList;
typedef std::map<MarkerId,MarkerEstimate> MarkerIdToEstimate;

void getMarkerEstimate(
  Eigen::Affine3d const& T_WC,
  Eigen::Matrix<double,6,6> const& cov_T_WC,
  DetectedMarker const& detected_marker,
  MarkerEstimate* marker_estimate);

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
