#ifndef COMMON_MATHS_HPP
#define COMMON_MATHS_HPP

#include <eigen3/Eigen/Dense>

namespace common {

// Degree <-> Radian conversions
double convertDegreeToRadian(double angle_deg);
double convertRadianToDegree(double angle_rad);

// Exponential and Logarithm mapping for SO3*R3 Lie group
Eigen::Affine3d expMap(Eigen::Matrix<double,6,1> const& tau);
Eigen::Matrix<double,6,1> logMap(Eigen::Affine3d const& T);
Eigen::Matrix3d skew(Eigen::Vector3d const& w);

// Left Jacobian on SO3
Eigen::Matrix3d leftJacobianSO3(Eigen::Vector3d const& theta);

// Pose Jacobians on the SO3*R3 Lie group
Eigen::Matrix<double,6,6> leftProductJacobian(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2);
Eigen::Matrix<double,6,6> rightProductJacobian(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2);
Eigen::Matrix<double,6,6> inverseJacobian(Eigen::Affine3d const& T);

} // namespace common

#endif // COMMON_MATHS_HPP
