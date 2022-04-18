#include <iostream>

#include <common/maths.hpp>
#include <common/pose_sampler.hpp>

int main(int argc, char* argv[])
{
  // Check the identity transform
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,1> tau = common::so3r3::logMap(T);
  T = common::so3r3::expMap(tau);
  std::cout << "T:\n" << T.matrix() << std::endl;

  /* Checking Jacobians
   * f(tau + T) = Exp( J*tau ) * f(T)
   * => J*tau = Log( f(tau + T) * f(T)^-1 )
   * => vérifier l'égalité de ces deux quantités
   * Attention, Exp, Log et * sont définis sur SO3*R3, pas SE3
   */

  // Initialize the pose sampler
  common::PoseSampler sampler(Eigen::Affine3d::Identity(), 10, M_PI);
  Eigen::Affine3d T1 = sampler.sample();
  Eigen::Affine3d T2 = sampler.sample();

  // Check the left pose product Jacobian matrix
  Eigen::Matrix<double,6,6> const J = common::so3r3::leftSe3ProductJacobian(T1,T2);
  tau = 1e-1 * Eigen::Matrix<double,6,1>::Ones();
  Eigen::Affine3d T1T2 = T1*T2;
  Eigen::Affine3d tauT1T2 = common::so3r3::product(tau,T1)*T2;
  Eigen::Matrix<double,6,1> e = common::so3r3::logMap( 
    common::so3r3::product( tauT1T2, common::so3r3::inverse(T1T2) ) ) - J*tau;
  std::cout << "Error: " << e.transpose() << std::endl;

  // Check the right pose product Jacobian matrix
  //~ J = common::rightSe3ProductJacobian(T1, T2);
  //~ std::cout << "J:\n" << J << std::endl;

  // Check the pose inverse Jacobian matrix
  //~ J = common::se3InverseJacobian(T1);
  //~ std::cout << "J:\n" << J << std::endl;

  return EXIT_SUCCESS;
}
