#ifndef COMMON_YAML_SERIALIZATION_HPP
#define COMMON_YAML_SERIALIZATION_HPP

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace common {
namespace yaml_serialization {

  Eigen::Affine3d deserializePose(YAML::Node const& node);

  Eigen::Matrix<double,6,6> deserializePoseCovariance(YAML::Node const& node);

} // namespace yaml_serialization
} // namespace common

#endif // COMMON_YAML_SERIALIZATION_HPP
