#include <cstdlib>
#include <iostream>

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Marker::Marker(
  Eigen::Affine3d const& T_WC,
  Eigen::Matrix<double,6,6> const& cov_T_WC,
  DetectedMarker const& detected_marker)
{
  // Get the tag ID and family
  this->id = static_cast<MarkerId>(detected_marker.marker_id);
  this->family = getMarkerFamily(this->id);
  
  // Get the timestamp in nanoseconds
  TimePoint const timestamp = Time::now();
  this->timestamp_ns = convertToNanoseconds(timestamp);
  
  // Get the estimate of the tag
  Eigen::Affine3d const& T_CM = detected_marker.T_CM;
  Eigen::Matrix<double,6,6> const& cov_T_CM = detected_marker.cov_T_CM;
  this->T_WM = T_WC * T_CM;
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TWC = so3r3::leftSe3ProductJacobian(T_WC, T_CM);
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TCM = so3r3::rightSe3ProductJacobian(T_WC, T_CM);
  this->cov_T_WM =
    J_TWM_wrt_TWC * cov_T_WC * J_TWM_wrt_TWC.transpose() +
    J_TWM_wrt_TCM * cov_T_CM * J_TWM_wrt_TCM.transpose();
}

//--------------------------------------------------------------------------------------------------
// Methods
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

bool Marker::serialize(
  std::vector<char>::iterator it_begin,
  std::vector<char>::iterator it_end) const
{
  // Check the pointer message
  size_t const distance = std::distance(it_begin, it_end);
  if(distance != Marker::MESSAGE_SIZE_BYTES) return false;
  
  // Serialize the header
  std::vector<char>::iterator byte_it = it_begin;
  std::memcpy(&(*byte_it), &id, sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&(*byte_it), &family, sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&(*byte_it), &timestamp_ns, sizeof(int64_t));
  byte_it += sizeof(int64_t);
  
  // Serialize the pose
  Eigen::Matrix<double,7,1> pose;
  pose.head<4>() = Eigen::Quaterniond(this->T_WM.rotation()).coeffs();
  pose.tail<3>() = T_WM.translation();
  std::memcpy(&(*byte_it), pose.data(), 7*sizeof(double));
  byte_it += 7*sizeof(double);
  
  // Serialize the covariance matrix
  //~ std::vector<double> covariance(21);
  //~ int cov_idx = 0;
  //~ for(int i=0; i<6; i++)
    //~ for(int j=i; j<6; j++)
      //~ covariance[cov_idx++] = this->cov_T_WM(i,j);
  //~ std::memcpy(&(*byte_it), covariance.data(), 21*sizeof(double));
  //~ byte_it += 21*sizeof(double);

  bool success = (byte_it == it_end);  
  return success;
}

//--------------------------------------------------------------------------------------------------

bool Marker::deserialize(
  std::vector<char>::const_iterator it_begin,
  std::vector<char>::const_iterator it_end)
{
  // Check the length of the message
  size_t const distance = std::distance(it_begin, it_end);
  if(distance != Marker::MESSAGE_SIZE_BYTES) return false;
  
  // Deserialize the header
  std::vector<char>::const_iterator byte_it = it_begin;
  std::memcpy(&this->id, &(*byte_it), sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&this->family, &(*byte_it), sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&this->timestamp_ns, &(*byte_it), sizeof(uint64_t));
  byte_it += sizeof(int64_t);
  
  // Deserialize the pose
  Eigen::Matrix<double,7,1> pose;
  std::memcpy(pose.data(), &(*byte_it), 7*sizeof(double));
  byte_it += 7*sizeof(double);
  this->T_WM.translation() = pose.tail<3>();
  this->T_WM.linear() = Eigen::Quaterniond(pose.head<4>()).normalized().toRotationMatrix();
  
  // Deserialize the covariance
  //~ std::vector<double> covariance(21);
  //~ std::memcpy(covariance.data(), &(*byte_it), 21*sizeof(double));
  //~ int cov_idx = 0;
  //~ for(int i=0; i<6; i++)
  //~ {
    //~ for(int j=i; j<6; j++)
    //~ {
      //~ this->cov_T_WM(i,j) = covariance[cov_idx];
      //~ this->cov_T_WM(j,i) = covariance[cov_idx];
      //~ cov_idx += 1;
    //~ }
  //~ }
  //~ byte_it += 21*sizeof(double);

  bool success = (byte_it == it_end);
  return success;
}

//--------------------------------------------------------------------------------------------------

bool Marker::serialize(MarkerIdToEstimate const& markers, std::vector<char>* serialized_markers_ptr)
{
  // Initialize the buffer for serialized markers
  CHECK_NOTNULL(serialized_markers_ptr);
  std::vector<char>& serialized_markers = *serialized_markers_ptr;
  size_t num_markers = markers.size();
  serialized_markers.resize(num_markers * Marker::MESSAGE_SIZE_BYTES);
  
  // Fill the buffer for serialized markers
  std::vector<char>::iterator byte_it = serialized_markers.begin();
  for(MarkerIdToEstimate::value_type const& pair : markers)
  {
    Marker const& marker = pair.second;
    std::vector<char>::iterator next_it = std::next(byte_it, Marker::MESSAGE_SIZE_BYTES);
    marker.serialize(byte_it, next_it);
    byte_it += Marker::MESSAGE_SIZE_BYTES;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Marker::deserialize(std::vector<char> const& message, MarkerIdToEstimate* markers_ptr)
{
  // Get the number of markers to deserialize
  size_t const message_size = message.size();
  CHECK(message_size % Marker::MESSAGE_SIZE_BYTES == 0);
  size_t num_markers = message_size / Marker::MESSAGE_SIZE_BYTES;
  if(markers_ptr == NULL) return false;
  MarkerIdToEstimate& markers = *markers_ptr;
  
  // Deserialize the markers
  std::vector<char>::const_iterator byte_it = message.cbegin();
  for(size_t marker_idx=0u; marker_idx<num_markers; ++marker_idx)
  {
    Marker marker;
    std::vector<char>::const_iterator next_it = std::next(byte_it,Marker::MESSAGE_SIZE_BYTES);
    marker.deserialize(byte_it, next_it);
    markers[marker.id] = marker;
    byte_it = next_it;
  }
  
  return true;
}

//--------------------------------------------------------------------------------------------------

std::string Marker::print() const
{
  std::stringstream out;
  
  out << "Marker:" << std::endl;
  out << "- id: " << static_cast<int>(this->id) << std::endl;
  out << "- family: ";
  switch(this->family)
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
  out << "- timestamp: " << this->timestamp_ns << std::endl;
  out << "- T_WM: " << std::endl << this->T_WM.matrix() << std::endl;
  out << "- cov_T_WM: " << std::endl << this->cov_T_WM.matrix() << std::endl;
  
  return out.str();
}

//--------------------------------------------------------------------------------------------------

MarkerId Marker::sampleMarkerId(MarkerFamily marker_family)
{
  MarkerId marker_id;
  switch(marker_family)
  {
    case common::MarkerFamily::CENTRAL_MARKER:
      marker_id = 42;
      break;
    case common::MarkerFamily::PURPLE_TEAM_MARKER:
      // Marker ids from 51 to 70
      marker_id = 51 + (std::rand()%20);
      break;
    case common::MarkerFamily::PURPLE_TEAM_ROBOT:
      // Marker ids from 1 to 5
      marker_id = 1 + (std::rand()%5);
      break;
    case common::MarkerFamily::ROCK_SAMPLE:
      marker_id = 17;
      break;
    case common::MarkerFamily::TREASURE_BLUE_SAMPLE:
      marker_id = 13;
      break;
    case common::MarkerFamily::TREASURE_GREEN_SAMPLE:
      marker_id = 36;
      break;
    case common::MarkerFamily::TREASURE_RED_SAMPLE:
      marker_id = 47;
      break;
    case common::MarkerFamily::YELLOW_TEAM_MARKER:
      // Marker ids from 71 to 91
      marker_id = 71 + (std::rand()%20);
      break;
    case common::MarkerFamily::YELLOW_TEAM_ROBOT:
      // Marker ids from 6 to 10
      marker_id = 6 + (std::rand()%5);
      break;
    case common::MarkerFamily::UNKNOWN:
    default:
      throw std::runtime_error("Unknown marker family");
  }
  return marker_id;
}

//--------------------------------------------------------------------------------------------------

} // namespace common
