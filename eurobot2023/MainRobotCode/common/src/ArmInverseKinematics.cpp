#include <cmath>
#include <iostream>

#include <common/ArmInverseKinematics.hpp>

#define ARM_INVERSE_KINEMATICS_TOLERANCE 1e-2

namespace common {

bool arm_inverse_kinematics(
  double r, double theta_rad, double z, double psi_rad,
  std::array<double,4>* angles_ptr)
{
  // Arm parameters
  double constexpr d12 = 76.5e-3;
  double constexpr d23 = 70.6e-3;
  double constexpr d34 = 90.6e-3;
  double constexpr d45x = 29.1e-3;
  double constexpr d45z = 45.8e-3;

  // The first angle theta1 is directly given by theta
  std::array<double,4>& angles = *angles_ptr;
  angles[0] = theta_rad;

  // The theta2 and theta3 angles are deduced from a plane inverse kinematics problem.
  // The required position matches the center of the suction cup. We compute the polar coordinates
  // of the corresponding targeted point on the previous piece.
  double r0 = r - d12 - d45x*std::cos(psi_rad) + d45z*std::sin(psi_rad);
  double z0 = z - d45x*std::sin(psi_rad) - d45z*std::cos(psi_rad);

  // Check that the problem is consistent
  double constexpr r0max = d23 + d34;
  if(r0>r0max) 
  {
    std::cout << "Error ArmInverseKinematics : r0>r0max" << std::endl;
    return false;
  }


  // Solve the plane inverse kinematics problem
  bool success = false;
  double const l2 = z0*z0 + r0*r0;
  double const psi0_rad = std::atan2(z0, r0);
  double cos_theta23 = (l2 - d23*d23 - d34*d34)/(2*d23*d34);
  cos_theta23 = std::max(-1., std::min(cos_theta23, 1.));
  double theta23 = - std::acos( cos_theta23 );

  double r_error;
  double z_error;
  for(int i=0; i<=1; i+=1)
  {
    // Compute angles and errors
    double theta12 = psi0_rad - std::asin( d34*std::sin(theta23)/std::sqrt(l2) );
    r_error = r0 - d23*std::cos(theta12) - d34*std::cos(theta12+theta23);
    z_error = z0 - d23*std::sin(theta12) - d34*std::sin(theta12+theta23);

    // Check success conditions are met
    double constexpr theta23_max = -0.05;
    double constexpr theta23_min = - 2.5;
    bool const ok1 = std::max(std::fabs(r_error),std::fabs(z_error))<ARM_INVERSE_KINEMATICS_TOLERANCE;
    bool const ok2 = theta23<=theta23_max && theta23>=theta23_min;
    if(ok1 && ok2) {
      angles[1] = theta12;
      angles[2] = theta23;
      angles[3] = psi_rad - theta12 - theta23;
      success = true;
      break;
    } else {
      theta23 = - theta23;
      continue;
    }
  }

  if (!success)
  {
    std::cout << "Error ArmInverseKinematics : not success" << std::endl;
    std::cout << "r_error " << r_error << std::endl;
    std::cout << "z_error " << z_error << std::endl;
  }

  return success;
}

} // namespace common
