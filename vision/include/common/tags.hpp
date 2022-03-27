#ifndef COMMON_TAGS_HPP
#define COMMON_TAGS_HPP

#include <map>

#include <eigen3/Eigen/Dense>

namespace common {

//--------------------------------------------------------------------------------------------------
// Structure/Enum declarations
//--------------------------------------------------------------------------------------------------

typedef uint8_t TagId;
enum class TagFamily {
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
}; // enum class TagFamily

TagFamily getTagFamily(TagId id);

struct ArucoTag {
  TagId id = 0;
  TagFamily family = TagFamily::UNKNOWN;
  // Timestamp
  Eigen::Affine3d T_WM = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,6> cov_T_WM = Eigen::Matrix<double,6,6>::Identity();
}; // class ArucoTag

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
