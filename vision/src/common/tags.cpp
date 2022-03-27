#include <common/maths.hpp>
#include <common/tags.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------

MarkerFamily getMarkerFamily(MarkerId requested_marker_id)
{
  // Initialize the correspondence table when the function is first called.
  typedef std::map<MarkerId,MarkerFamily> MarkerIdToFamily;
  static std::map<MarkerId,MarkerFamily> marker_id_to_family;
  if(marker_id_to_family.empty())
  {
    // Markers appended to the robots
    for(MarkerId marker_id=1u; marker_id<=5u; marker_id++)
      marker_id_to_family[marker_id] = MarkerFamily::PURPLE_TEAM_ROBOT;
    for(MarkerId marker_id=6u; marker_id<=10u; marker_id++)
      marker_id_to_family[marker_id] = MarkerFamily::YELLOW_TEAM_ROBOT;
      
    // Markers reserved for the board playing area
    marker_id_to_family[13u] = MarkerFamily::TREASURE_BLUE_SAMPLE;
    marker_id_to_family[17u] = MarkerFamily::ROCK_SAMPLE;
    marker_id_to_family[36u] = MarkerFamily::TREASURE_GREEN_SAMPLE;
    marker_id_to_family[42u] = MarkerFamily::CENTRAL_MARKER;
    marker_id_to_family[47u] = MarkerFamily::TREASURE_RED_SAMPLE;
    
    // Markers reserved for the purple team
    for(MarkerId marker_id=51u; marker_id<=70u; marker_id++)
      marker_id_to_family[marker_id] = MarkerFamily::PURPLE_TEAM_MARKER;
    for(MarkerId marker_id=71u; marker_id<=90u; marker_id++)
      marker_id_to_family[marker_id] = MarkerFamily::YELLOW_TEAM_MARKER;
  }
  
  // Return the requested tag's family
  MarkerIdToFamily::const_iterator it = marker_id_to_family.find(requested_marker_id);
  MarkerFamily const marker_family = (it != marker_id_to_family.cend()) ? it->second : MarkerFamily::UNKNOWN;
  return marker_family;
}

//--------------------------------------------------------------------------------------------------

void getMarkerEstimate(
  Eigen::Affine3d const& T_WC,
  Eigen::Matrix<double,6,6> const& cov_T_WC,
  DetectedMarker const& detected_marker,
  MarkerEstimate* marker_estimate_ptr)
{
  MarkerEstimate& marker_estimate = *marker_estimate_ptr;
  
  // Get the tag ID and family
  marker_estimate.id = static_cast<MarkerId>(detected_marker.marker_id);
  marker_estimate.family = getMarkerFamily(marker_estimate.id);
  
  // Get the timestamp in nanoseconds
  TimePoint const timestamp = Time::now();
  marker_estimate.timestamp_ns = convertToNanoseconds(timestamp);
  
  // Get the estimate of the tag
  Eigen::Affine3d const& T_CM = detected_marker.T_CM;
  Eigen::Matrix<double,6,6> const& cov_T_CM = detected_marker.cov_T_CM;
  marker_estimate.T_WM = T_WC * T_CM;
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TWC = leftProductJacobian(T_WC, T_CM);
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TCM = rightProductJacobian(T_WC, T_CM);
  marker_estimate.cov_T_WM =
    J_TWM_wrt_TWC * cov_T_WC * J_TWM_wrt_TWC.transpose() +
    J_TWM_wrt_TCM * cov_T_CM * J_TWM_wrt_TCM.transpose();
}

//--------------------------------------------------------------------------------------------------

} // namespace common
