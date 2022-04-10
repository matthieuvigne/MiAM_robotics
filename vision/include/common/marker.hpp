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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // DetectedMarker
typedef std::vector<DetectedMarker> DetectedMarkerList;

//--------------------------------------------------------------------------------------------------
// Marker class
//--------------------------------------------------------------------------------------------------

class Marker;
typedef std::vector<Marker> MarkerList;
typedef std::map<MarkerId,Marker> MarkerIdToEstimate;

class Marker {

  public:
    Marker() = default;
    Marker(
      Eigen::Affine3d const& T_WC,
      Eigen::Matrix<double,6,6> const& cov_T_WC,
      DetectedMarker const& detected_marker);
  virtual ~Marker() = default;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    bool serialize(
      std::vector<char>::iterator it_begin,
      std::vector<char>::iterator it_end) const;
    bool deserialize(
      std::vector<char>::const_iterator it_begin,
      std::vector<char>::const_iterator it_end);
    std::string print() const;

  public:
    MarkerId id = 0;
    MarkerFamily family = MarkerFamily::UNKNOWN;
    int64_t timestamp_ns = 0u;
    Eigen::Affine3d T_WM = Eigen::Affine3d::Identity();
    Eigen::Matrix<double,6,6> cov_T_WM = Eigen::Matrix<double,6,6>::Identity();

  public:
    static bool serialize(MarkerIdToEstimate const& markers, std::vector<char>* message);
    static bool deserialize(std::vector<char> const& message, MarkerIdToEstimate* markers);
    static int constexpr MESSAGE_SIZE_BYTES =  1 * sizeof(MarkerId)     + /*Marker id*/
                                               1 * sizeof(MarkerFamily) + /*Marker family*/
                                               1 * sizeof(int64_t)      + /*timestamp*/
                                               7 * sizeof(double)       + /*Pose*/
                                              21 * sizeof(double);        /*Covariance*/

}; // class Marker

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
