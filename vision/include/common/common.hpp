#ifndef COMMON_COMMON_HPP
#define COMMON_COMMON_HPP

#include <eigen3/Eigen/Dense>

namespace common {

enum class Team {
  UNKNOWN,
  PURPLE,
  YELLOW
}; // enum class Team

Eigen::Affine3d getTWM();

} // namespace common

#endif // COMMON_COMMON_HPP
