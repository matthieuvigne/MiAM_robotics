#include <cstdlib>
#include <iostream>

#include <common/macros.hpp>
#include <common/marker.hpp>
#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Marker::Marker()
: status_       (Status::INVALID),
  id_           (0),
  family_       (Family::UNKNOWN),
  timestamp_ns_ (0),
  corners_      (),
  TCM_          (nullptr),
  cov_TCM_      (nullptr),
  TWM_          (nullptr),
  cov_TWM_      (nullptr)
{}

//--------------------------------------------------------------------------------------------------

Marker::Marker(Id id)
: Marker()
{
  id_ = id;
  family_ = getMarkerFamily(id_);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Marker::addMeasurement(
  int64_t timestamp_ns,
  std::vector<cv::Point2f> const& corners,
  Eigen::Affine3d const& TCM,
  Eigen::Matrix<double,6,6> const& cov_TCM)
{
  timestamp_ns_ = timestamp_ns;
  corners_ = corners;
  TCM_.reset(new Eigen::Affine3d(TCM));
  cov_TCM_.reset(new Eigen::Matrix<double,6,6>(cov_TCM));
  status_ = Status::MEASURED;
}

//--------------------------------------------------------------------------------------------------

void Marker::addEstimate(
  Eigen::Affine3d const& TWM,
  Eigen::Matrix<double,6,6> const& cov_TWM)
{
  TWM_.reset(new Eigen::Affine3d(TWM));
  cov_TWM_.reset(new Eigen::Matrix<double,6,6>(cov_TWM));
  status_ = Status::ESTIMATED;
}

//--------------------------------------------------------------------------------------------------

void Marker::estimateFromCameraPose(
  Eigen::Affine3d const& TWC,
  Eigen::Matrix<double,6,6> const& cov_TWC)
{
  // Compute the global pose of the marker
  CHECK(status_ == Status::MEASURED);
  Eigen::Affine3d const& TCM = *TCM_;
  TWM_.reset(new Eigen::Affine3d(TWC*TCM));

  // Compute the covariance of the marker
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TWC = so3r3::leftSe3ProductJacobian(TWC, TCM);
  Eigen::Matrix<double,6,6> const J_TWM_wrt_TCM = so3r3::rightSe3ProductJacobian(TWC, TCM);
  cov_TWM_.reset(new Eigen::Matrix<double,6,6>);
  *cov_TWM_ = J_TWM_wrt_TWC * cov_TWC * J_TWM_wrt_TWC.transpose()
            + J_TWM_wrt_TCM * (*cov_TCM_) * J_TWM_wrt_TCM.transpose();
  
  // Update the status of the marker
  status_ = Status::ESTIMATED;
}

//--------------------------------------------------------------------------------------------------

Marker::Family Marker::getMarkerFamily(Id requested_marker_id)
{
  // Initialize the correspondence table when the function is first called.
  typedef std::map<Id,Family> MarkerIdToFamily;
  static MarkerIdToFamily marker_id_to_family;
  if(marker_id_to_family.empty())
  {
    // Markers appended to the robots
    for(Id marker_id=1u; marker_id<=5u; marker_id++)
      marker_id_to_family[marker_id] = Family::PURPLE_TEAM_ROBOT;
    for(Id marker_id=6u; marker_id<=10u; marker_id++)
      marker_id_to_family[marker_id] = Family::YELLOW_TEAM_ROBOT;
      
    // Markers reserved for the board playing area
    marker_id_to_family[13u] = Family::TREASURE_BLUE_SAMPLE;
    marker_id_to_family[17u] = Family::ROCK_SAMPLE;
    marker_id_to_family[36u] = Family::TREASURE_GREEN_SAMPLE;
    marker_id_to_family[42u] = Family::CENTRAL_MARKER;
    marker_id_to_family[47u] = Family::TREASURE_RED_SAMPLE;
    
    // Markers reserved for the purple team
    for(Id marker_id=51u; marker_id<=70u; marker_id++)
      marker_id_to_family[marker_id] = Family::PURPLE_TEAM_MARKER;
    for(Id marker_id=71u; marker_id<=90u; marker_id++)
      marker_id_to_family[marker_id] = Family::YELLOW_TEAM_MARKER;
  }
  
  // Return the requested tag's family
  MarkerIdToFamily::const_iterator it = marker_id_to_family.find(requested_marker_id);
  Family const marker_family = (it != marker_id_to_family.cend()) ? it->second : Family::UNKNOWN;
  return marker_family;
}

//--------------------------------------------------------------------------------------------------

bool Marker::serialize(
  std::vector<char>::iterator it_begin,
  std::vector<char>::iterator it_end) const
{
  // Check the pointer message
  size_t const distance = std::distance(it_begin, it_end);
  CHECK(distance == Marker::MESSAGE_SIZE_BYTES);
  
  // Serialize the header
  std::vector<char>::iterator byte_it = it_begin;
  std::memcpy(&(*byte_it), &id_, sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&(*byte_it), &family_, sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&(*byte_it), &timestamp_ns_, sizeof(int64_t));
  byte_it += sizeof(int64_t);
  
  // Serialize the pose
  Eigen::Matrix<double,7,1> pose;
  if(TWM_)
  {
    pose.head<4>() = Eigen::Quaterniond(TWM_->rotation()).coeffs();
    pose.tail<3>() = TWM_->translation();
  }
  else
  {
    // If not initialize => identity pose
    pose << 0., 0., 0., 1., 0., 0., 0.;
  }
  std::memcpy(&(*byte_it), pose.data(), 7*sizeof(double));
  byte_it += 7*sizeof(double);
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
  CHECK(distance == Marker::MESSAGE_SIZE_BYTES);
  
  // Deserialize the header
  std::vector<char>::const_iterator byte_it = it_begin;
  std::memcpy(&id_, &(*byte_it), sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&family_, &(*byte_it), sizeof(uint8_t));
  byte_it += sizeof(uint8_t);
  std::memcpy(&timestamp_ns_, &(*byte_it), sizeof(uint64_t));
  byte_it += sizeof(int64_t);
  
  // Deserialize the pose
  Eigen::Matrix<double,7,1> pose;
  std::memcpy(pose.data(), &(*byte_it), 7*sizeof(double));
  byte_it += 7*sizeof(double);
  TWM_.reset(new Eigen::Affine3d);
  TWM_->translation() = pose.tail<3>();
  TWM_->linear() = Eigen::Quaterniond(pose.head<4>()).normalized().toRotationMatrix();
  bool success = (byte_it == it_end);
  return success;
}

//--------------------------------------------------------------------------------------------------

bool Marker::serialize(MarkerEstimates const& estimates, std::vector<char>* message_ptr)
{
  // Initialize the buffer for serialized markers
  CHECK_NOTNULL(message_ptr);
  std::vector<char>& message = *message_ptr;
  size_t num_markers = estimates.size();
  message.resize(num_markers * Marker::MESSAGE_SIZE_BYTES);
  
  // Fill the buffer for serialized markers
  std::vector<char>::iterator byte_it = message.begin();
  for(MarkerEstimates::value_type const& pair : estimates)
  {
    Marker const& marker = pair.second;
    std::vector<char>::iterator next_it = std::next(byte_it, Marker::MESSAGE_SIZE_BYTES);
    marker.serialize(byte_it, next_it);
    byte_it += Marker::MESSAGE_SIZE_BYTES;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Marker::deserialize(std::vector<char> const& message, MarkerEstimates* estimates_ptr)
{
  // Check the deserialization target
  CHECK_NOTNULL(estimates_ptr);
  MarkerEstimates& estimates = *estimates_ptr;

  // Get the number of markers to deserialize
  size_t const message_size = message.size();
  CHECK(message_size % Marker::MESSAGE_SIZE_BYTES == 0);
  size_t num_markers = message_size / Marker::MESSAGE_SIZE_BYTES;
  
  // Deserialize the markers
  std::vector<char>::const_iterator byte_it = message.cbegin();
  for(size_t marker_idx=0u; marker_idx<num_markers; ++marker_idx)
  {
    Marker marker;
    std::vector<char>::const_iterator next_it = std::next(byte_it,Marker::MESSAGE_SIZE_BYTES);
    marker.deserialize(byte_it, next_it);
    estimates.insert(std::make_pair(marker.timestamp_ns_, marker));
    byte_it = next_it;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

std::string Marker::print() const
{
  std::stringstream out;
  
  out << "Marker:" << std::endl;
  out << "- id: " << static_cast<int>(id_) << std::endl;
  out << "- family: ";
  switch(family_)
  {
    case Family::CENTRAL_MARKER:
      out << "CENTRAL_MARKER";
      break;
    case Family::PURPLE_TEAM_MARKER:
      out << "PURPLE_TEAM_MARKER";
      break;
    case Family::PURPLE_TEAM_ROBOT:
      out << "PURPLE_TEAM_ROBOT";
      break;
    case Family::ROCK_SAMPLE:
      out << "ROCK_SAMPLE";
      break;
    case Family::TREASURE_BLUE_SAMPLE:
      out << "TREASURE_BLUE_SAMPLE";
      break;
    case Family::TREASURE_GREEN_SAMPLE:
      out << "TREASURE_GREEN_SAMPLE";
      break;
    case Family::TREASURE_RED_SAMPLE:
      out << "TREASURE_RED_SAMPLE";
      break;
    case Family::UNKNOWN:
      out << "UNKNOWN";
      break;
    case Family::YELLOW_TEAM_MARKER:
      out << "YELLOW_TEAM_MARKER";
      break;
    case Family::YELLOW_TEAM_ROBOT:
      out << "YELLOW_TEAM_ROBOT";
      break;
    default:
      out << "No tag";
      break;
  }
  out << std::endl;
  out << "- timestamp: " << timestamp_ns_ << std::endl;
  if(TWM_)      out << "- T_WM: " << std::endl << TWM_->matrix() << std::endl;
  if(cov_TWM_)  out << "- cov_T_WM: " << std::endl << cov_TWM_->matrix() << std::endl;
  
  return out.str();
}

//--------------------------------------------------------------------------------------------------

Marker::Id Marker::sampleMarkerId(Family marker_family)
{
  Marker::Id marker_id;
  switch(marker_family)
  {
    case Family::CENTRAL_MARKER:
      marker_id = 42;
      break;
    case Family::PURPLE_TEAM_MARKER:
      // Marker ids from 51 to 70
      marker_id = 51 + (std::rand()%20);
      break;
    case Family::PURPLE_TEAM_ROBOT:
      // Marker ids from 1 to 5
      marker_id = 1 + (std::rand()%5);
      break;
    case Family::ROCK_SAMPLE:
      marker_id = 17;
      break;
    case Family::TREASURE_BLUE_SAMPLE:
      marker_id = 13;
      break;
    case Family::TREASURE_GREEN_SAMPLE:
      marker_id = 36;
      break;
    case Family::TREASURE_RED_SAMPLE:
      marker_id = 47;
      break;
    case Family::YELLOW_TEAM_MARKER:
      // Marker ids from 71 to 91
      marker_id = 71 + (std::rand()%20);
      break;
    case Family::YELLOW_TEAM_ROBOT:
      // Marker ids from 6 to 10
      marker_id = 6 + (std::rand()%5);
      break;
    case Family::UNKNOWN:
    default:
      throw std::runtime_error("Unknown marker family");
  }
  return marker_id;
}

//--------------------------------------------------------------------------------------------------

bool Marker::isUnique() const
{
  bool is_unique = true;
  switch(family_)
  {
    case Family::ROCK_SAMPLE:
    case Family::TREASURE_RED_SAMPLE:
    case Family::TREASURE_GREEN_SAMPLE:
    case Family::TREASURE_BLUE_SAMPLE:
      is_unique = false;
      break;
    default:
      is_unique = true;
      break;
  }
  return is_unique;
}

//--------------------------------------------------------------------------------------------------

} // namespace common
