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
   * => J*tau = Log( f(tau + T) * f(T)^(-1) )
   * Beware, Exp, Log et * are defined over SO3*R3, not SE3
   */

  // Initialize the pose sampler
  common::PoseSampler sampler(Eigen::Affine3d::Identity(), 10, M_PI);
  Eigen::Affine3d T1 = sampler.sample();
  Eigen::Affine3d T2 = sampler.sample();

  // Check the left SE3 pose product Jacobian matrix
  Eigen::Matrix<double,6,6> J = common::so3r3::leftSe3ProductJacobian(T1,T2);
  tau = 1e-2 * Eigen::Matrix<double,6,1>::Ones();
  Eigen::Affine3d const tT1 = common::so3r3::product(tau,T1);
  Eigen::Affine3d const T1T2 = T1*T2;
  Eigen::Affine3d const tT1T2 = tT1*T2;
  Eigen::Affine3d const invT1T2 = common::so3r3::inverse(T1T2);
  Eigen::Matrix<double,6,1> residual = 
    common::so3r3::logMap( common::so3r3::product( tT1T2, invT1T2 ) ) - J*tau;
  std::cout << "Residual: " << residual.transpose() << std::endl;

  // Check the right pose product Jacobian matrix
  J = common::so3r3::rightSe3ProductJacobian(T1, T2);
  Eigen::Affine3d const tT2 = common::so3r3::product(tau,T2);
  Eigen::Affine3d const T1tT2 = T1*tT2;
  residual = common::so3r3::logMap( common::so3r3::product(T1tT2, invT1T2) ) - J*tau;
  std::cout << "Residual: " << residual.transpose() << std::endl;

  // Check the pose inverse Jacobian matrix
  J = common::so3r3::se3InverseJacobian(T1);
  Eigen::Affine3d const invtT1 = tT1.inverse();
  Eigen::Affine3d const invT1 = common::so3r3::inverse( T1.inverse() );
  residual = common::so3r3::logMap( common::so3r3::product(invtT1,invT1) ) - J*tau;
  std::cout << "Residual: " << residual.transpose() << std::endl;

  return EXIT_SUCCESS;
}
