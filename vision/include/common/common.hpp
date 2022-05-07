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
Eigen::Vector3d getWpCi(Team team);
Eigen::Quaterniond getqWR();
Eigen::Quaterniond getqRC(double azimuth_deg, double elevation_deg);
  
} // namespace common

#endif // COMMON_COMMON_HPP
