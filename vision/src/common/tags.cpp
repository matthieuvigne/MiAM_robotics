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

bool serializeMarker(MarkerEstimate const& marker, std::vector<unsigned char>* message_ptr)
{
  // Check the pointer message
  if(message_ptr == 0) return false;
  std::vector<unsigned char>& message = *message_ptr;
  message.clear();
  
  // Get the size of the message
  int constexpr message_size =  1 * sizeof(uint8_t) + /*Marker id*/
                                1 * sizeof(uint8_t) + /*Marker family*/
                                1 * sizeof(int64_t) + /*timestamp*/
                                7 * sizeof(double)  + /*Pose*/
                               21 * sizeof(double)  + /*Covariance*/
                               sizeof(char);          /*NULL termination*/
  message.resize(message_size);
  int byte_idx = 0;
  
  // Serialize the header
  std::memcpy(&message[byte_idx], &marker.id, sizeof(uint8_t));
  byte_idx += sizeof(uint8_t);
  std::memcpy(&message[byte_idx], &marker.family, sizeof(uint8_t));
  byte_idx += sizeof(uint8_t);
  std::memcpy(&message[byte_idx], &marker.timestamp_ns, sizeof(int64_t));
  byte_idx += sizeof(int64_t);
  
  // Serialize the pose
  Eigen::Matrix<double,7,1> pose;
  pose.head<4>() = Eigen::Quaterniond(marker.T_WM.rotation()).coeffs();
  pose.tail<3>() = marker.T_WM.translation();
  std::memcpy(&message[byte_idx], pose.data(), 7*sizeof(double));
  byte_idx += 7*sizeof(double);
  
  // Serialize the covariance matrix
  std::vector<double> covariance(21);
  int cov_idx = 0;
  for(int i=0; i<6; i++)
    for(int j=i; j<6; j++)
      covariance[cov_idx++] = marker.cov_T_WM(i,j);
  std::memcpy(&message[byte_idx], covariance.data(), 21*sizeof(double));
  byte_idx += 21*sizeof(double);
  
  // Null terminated
  message[byte_idx] ='\0';
  
  return true;
}

//--------------------------------------------------------------------------------------------------

bool deserializeMarker(std::vector<unsigned char> const& message, MarkerEstimate* marker_ptr)
{
  // Check the length of the message
  if(marker_ptr == 0) return false;
  int constexpr message_size =  1 * sizeof(uint8_t) + /*Marker id*/
                                1 * sizeof(uint8_t) + /*Marker family*/
                                1 * sizeof(int64_t) + /*timestamp*/
                                7 * sizeof(double)  + /*Pose*/
                               21 * sizeof(double)  + /*Covariance*/
                                1 * sizeof(char);     /*NULL termination*/
  if(message.size() != message_size) return false;
  MarkerEstimate& marker = *marker_ptr;
  
  // Deserialize the header
  int byte_idx = 0;
  std::memcpy(&marker.id, &message[byte_idx], sizeof(uint8_t));
  byte_idx += sizeof(uint8_t);
  std::memcpy(&marker.family, &message[byte_idx], sizeof(uint8_t));
  byte_idx += sizeof(uint8_t);
  std::memcpy(&marker.timestamp_ns, &message[byte_idx], sizeof(uint64_t));
  byte_idx += sizeof(int64_t);
  
  // Deserialize the pose
  Eigen::Matrix<double,7,1> pose;
  std::memcpy(pose.data(), &message[byte_idx], 7*sizeof(double));
  byte_idx += 7*sizeof(double);
  marker.T_WM.translation() = pose.tail<3>();
  marker.T_WM.linear() = Eigen::Quaterniond(pose.head<4>()).normalized().toRotationMatrix();
  
  // Deserialize the covariance
  std::vector<double> covariance(21);
  std::memcpy(covariance.data(), &message[byte_idx], 21*sizeof(double));
  int cov_idx = 0;
  for(int i=0; i<6; i++)
  {
    for(int j=i; j<6; j++)
    {
      marker.cov_T_WM(i,j) = covariance[cov_idx];
      marker.cov_T_WM(j,i) = covariance[cov_idx];
      cov_idx += 1;
    }
  }
  
  return true;
}

//--------------------------------------------------------------------------------------------------

std::string printMarker(MarkerEstimate const& marker)
{
  std::stringstream out;
  
  out << "Marker:" << std::endl;
  out << "- id: " << static_cast<int>(marker.id) << std::endl;
  out << "- family: ";
  switch(marker.family)
  {
    case common::MarkerFamily::CENTRAL_MARKER:
      out << "CENTRAL_MARKER";
      break;
    case common::MarkerFamily::PURPLE_TEAM_MARKER:
      out << "PURPLE_TEAM_MARKER";
      break;
    case common::MarkerFamily::PURPLE_TEAM_ROBOT:
      out << "PURPLE_TEAM_ROBOT";
      break;
    case common::MarkerFamily::ROCK_SAMPLE:
      out << "ROCK_SAMPLE";
      break;
    case common::MarkerFamily::TREASURE_BLUE_SAMPLE:
      out << "TREASURE_BLUE_SAMPLE";
      break;
    case common::MarkerFamily::TREASURE_GREEN_SAMPLE:
      out << "TREASURE_GREEN_SAMPLE";
      break;
    case common::MarkerFamily::TREASURE_RED_SAMPLE:
      out << "TREASURE_RED_SAMPLE";
      break;
    case common::MarkerFamily::UNKNOWN:
      out << "UNKNOWN";
      break;
    case common::MarkerFamily::YELLOW_TEAM_MARKER:
      out << "YELLOW_TEAM_MARKER";
      break;
    case common::MarkerFamily::YELLOW_TEAM_ROBOT:
      out << "YELLOW_TEAM_ROBOT";
      break;
    default:
      out << "No tag";
      break;
  }
  out << std::endl;
  out << "- timestamp: " << marker.timestamp_ns << std::endl;
  out << "- T_WM: " << std::endl << marker.T_WM.matrix() << std::endl;
  out << "- cov_T_WM: " << std::endl << marker.cov_T_WM.matrix() << std::endl;
  
  return out.str();
}

//--------------------------------------------------------------------------------------------------

} // namespace common
