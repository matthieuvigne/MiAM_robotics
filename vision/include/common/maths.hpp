#ifndef COMMON_MATHS_HPP
#define COMMON_MATHS_HPP

#include <eigen3/Eigen/Dense>

#define RAD M_PI/180.
#define DEG 180./M_PI
#define CM  1e-2
#define MM  1e-3

namespace common {

// Degree <-> Radian conversions
double convertDegreeToRadian(double angle_deg);
double convertRadianToDegree(double angle_rad);

// Operations on SO3 and so3
Eigen::Matrix3d skew(Eigen::Vector3d const& w);
Eigen::Matrix3d leftJacobianSO3(Eigen::Vector3d const& theta);

namespace so3r3 {

// Exponential and Logarithm mapping for SO3*R3 Lie group
Eigen::Affine3d expMap(Eigen::Matrix<double,6,1> const& tau);
Eigen::Affine3d expMap(Eigen::Vector3d const& rotation, Eigen::Vector3d const& translation);
Eigen::Matrix<double,6,1> logMap(Eigen::Affine3d const& T);


// SO3*R3 product
Eigen::Affine3d product(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2);
Eigen::Affine3d product(Eigen::Matrix<double,6,1> const& tau, Eigen::Affine3d const& T);
Eigen::Affine3d inverse(Eigen::Affine3d const T);

// Box operators
Eigen::Affine3d boxplus(Eigen::Matrix<double,6,1> const& tau, Eigen::Affine3d const& T);
Eigen::Matrix<double,6,1> boxminus(Eigen::Affine3d const& T2, Eigen::Affine3d const& T1);

// Pose Jacobians for SE3 product on the SO3*R3 Lie group
Eigen::Matrix<double,6,6> leftSe3ProductJacobian(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2);
Eigen::Matrix<double,6,6> rightSe3ProductJacobian(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2);
Eigen::Matrix<double,6,6> se3InverseJacobian(Eigen::Affine3d const& T);
Eigen::Matrix<double,3,6> poseJacobian(Eigen::Affine3d const& T, Eigen::Vector3d const& p);

} // namespace so3r3

} // namespace common

#endif // COMMON_MATHS_HPP
