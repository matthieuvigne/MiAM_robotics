#ifndef COMMON_ARM_INVERSE_KINEMATICS_HPP
#define COMMON_ARM_INVERSE_KINEMATICS_HPP

#include <array>

namespace common {
  
bool arm_inverse_kinematics(
  double r, double theta_rad, double z, double psi_rad,
  std::array<double,4>* angles_ptr);

} // namespace common

#endif
