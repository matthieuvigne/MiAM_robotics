#include <cmath>
#include <iostream>

#include <common/ArmInverseKinematics.hpp>

namespace common {

bool arm_inverse_kinematics(
  double r, double theta_rad, double z,
  std::array<double,4>* angles_ptr)
{
  // The first angle theta1 is directly given by theta
  std::array<double,4>& angles = *angles_ptr;
  angles[0] = theta_rad;
  
  // The theta2 and theta3 angles are deduced from a plane inverse kinematics problem.
  // The required position matches the center of the suction cup. We compute the polar coordinates
  // of the corresponding targeted point on the previous piece.
  double constexpr d12x = 76.5e-3;
  double constexpr d45x = 29.1e-3;
  double constexpr d45z = 45.8e-3;
  double r4 = r - d45z - d12x;
  double z4 = z + d45x;
  
  // Check that the problem is consistent
  double constexpr d23 = 70.6e-3;
  double constexpr d34 = 90.6e-3;
  double constexpr rmax = d23 + d34;
  if(r4>rmax) return false;
  
  // Solve the plane inverse kinematics problem
  // Theta3 is solved by applying the cosine law to the plane problem
  //    cos(theta3) = (z4^2 + r4^2 - d23^2 - d34^2) / 2*d23*d34
  // Theta2 is deduced from the sine law applied to the 2D problem;
  //    atan(z4/r4) = theta2 + asin( d34*sin(theta3) / sqrt(r4^2 + z4^2))
  double constexpr theta3_max = 0;
  double constexpr theta3_min = - M_PI_2;
  double theta3 = std::acos((z4*z4 + r4*r4 - d23*d23 - d34*d34)/(2*d23*d34));
  if(theta3 < theta3_min || theta3 > theta3_max) theta3 *= -1;
  angles[2] = theta3;
  double theta2 = std::atan(z4/r4) - std::asin( (d34*std::sin(theta3))/(std::sqrt(z4*z4+r4*r4)) );
  angles[1] = theta2;
  
  // Compute theta4 to make the suction axe vertical
  angles[3] = - M_PI_2 - theta2 - theta3;
  
  return true;
}

} // namespace common
