#include <common/yaml_serialization.hpp>

namespace common {
namespace yaml_serialization {

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d deserializePose(YAML::Node const& node)
{
  Eigen::Affine3d pose;
  std::vector<double> data = node.as<std::vector<double>>();
  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> pose_matrix(data.data());
  pose.matrix() = pose_matrix;
  return pose;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> deserializePoseCovariance(YAML::Node const& node)
{
  std::vector<double> data = node.as<std::vector<double>>();
  Eigen::Map<Eigen::Matrix<double,6,1>> sigmas(data.data());
  sigmas.head<3>() *= M_PI/180.; // Conversion degrees -> radians
  Eigen::Matrix<double,6,6> covariance = sigmas.cwiseAbs2().asDiagonal();
  return covariance;
}

//--------------------------------------------------------------------------------------------------

} // namespace yaml_serialization
} // namespace common
