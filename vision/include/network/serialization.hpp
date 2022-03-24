#ifndef NETWORK_SERIALIZATION_HPP
#define NETWORK_SERIALIZATION_HPP

#include <string>

namespace network {
namespace serialization {

void serialize(
  std::string* message,
  int tag_number,
  Eigen::Affine3d T_WT);

void deserialize(
  std::string* message);

} // namespace serialization
} // namespace network

#endif // NETWORK_SERIALIZATION_HPP
