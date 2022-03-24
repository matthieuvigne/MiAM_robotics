#include <iostream>

#include <eigen3/Eigen/Dense>

int main(int argc, char* argv[])
{
  // Initialize quaternion
  Eigen::AngleAxisd const aa(M_PI_4, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond const q(aa);
  std::cout << q.coeffs().transpose() << std::endl;
  
  // Initialize the position
  Eigen::Vector3d const p(1,2,3);
  
  // Initialize the affine transformation
  Eigen::Affine3d T(q);
  T.pretranslate(p);
  std::cout << T.matrix() << std::endl;
  std::cout << T.rotation() << std::endl; 
  
  return EXIT_SUCCESS;
}
