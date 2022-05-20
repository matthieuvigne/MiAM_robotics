#ifndef COMMON_COMMON_HPP
#define COMMON_COMMON_HPP

#include <eigen3/Eigen/Dense>

namespace common {

enum class Team {
  UNKNOWN,
  PURPLE,
  YELLOW
}; // enum class Team

// Geometry functions
Eigen::Affine3d getTWM();
Eigen::Vector3d getWpRi(Team team);
Eigen::Quaterniond getqWR();
Eigen::Affine3d getTRC(double azimuth_deg, double elevation_deg,
  Eigen::Matrix<double,6,1>* J_TRC_wrt_azimuth = 0,
  Eigen::Matrix<double,6,1>* J_TRC_wrt_elevation = 0);
  
} // namespace common

#endif // COMMON_COMMON_HPP
