#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

double convertDegreeToRadian(double angle_deg)
{
  return M_PI*angle_deg/180.;
}

//--------------------------------------------------------------------------------------------------

double convertRadianToDegree(double angle_rad)
{
  return 180.*angle_rad/M_PI;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d skew(Eigen::Vector3d const& w)
{
  Eigen::Matrix3d S;
  S <<    0., -w.z(),  w.y(),
       w.z(),     0., -w.x(),
      -w.y(),  w.x(),     0.;
  return S;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d leftJacobianSO3(Eigen::Vector3d const& theta)
{
  double angle = theta.norm();
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  if (angle==0) return J;
  Eigen::Matrix3d const S = skew(theta);
  J += ((1-std::cos(angle))/angle)*S + ((angle-std::sin(angle))/std::pow(angle,3))*S*S;
  return J;
}

//--------------------------------------------------------------------------------------------------

} // namespace common

//--------------------------------------------------------------------------------------------------
// Methods for SO3*R3 parameterization
//--------------------------------------------------------------------------------------------------

namespace common {
namespace so3r3 {

Eigen::Matrix<double,6,6> leftSe3ProductJacobian(
  Eigen::Affine3d const& T1,
  Eigen::Affine3d const& T2)
{
  Eigen::Matrix3d const& R1 = T1.rotation();
  Eigen::Vector3d const& t2 = T2.translation();
  Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
  J.block<3,3>(3,0) = - skew(R1*t2);
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> rightSe3ProductJacobian(
  Eigen::Affine3d const& T1,
  Eigen::Affine3d const& T2)
{
  Eigen::Matrix3d const& R1 = T1.rotation();
  Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
  J.block<3,3>(0,0) = R1;
  J.block<3,3>(3,3) = R1;
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> se3InverseJacobian(
  Eigen::Affine3d const& T)
{
  Eigen::Vector3d const& t = T.translation();
  Eigen::Matrix3d const Rt = T.rotation().transpose();
  Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
  J.block<3,3>(0,0) = - Rt;
  J.block<3,3>(3,3) = - Rt;
  J.block<3,3>(3,0) = - Rt * skew(t);
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d expMap(Eigen::Matrix<double,6,1> const& tau)
{
  Eigen::Vector3d const rotation = tau.head<3>();
  Eigen::Vector3d const translation = tau.tail<3>();
  return expMap(rotation, translation);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d expMap(Eigen::Vector3d const& rotation, Eigen::Vector3d const& translation)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Vector3d const axis = rotation.normalized();
  double const angle = rotation.norm();
  Eigen::AngleAxisd const angle_axis(angle,axis);
  T.rotate(angle_axis);
  T.translation() = translation;
  return T;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,1> logMap(Eigen::Affine3d const& T)
{
  // Get the translational part
  Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Zero();
  tau.tail<3>() = T.translation();

  // Get the quaternion and check it
  Eigen::Quaterniond q(T.rotation());
  double q_norm = q.norm();
  if( q_norm < 1-1e-3 || q_norm > 1+1e-3)
    throw("Unormalized quaternion");
  Eigen::AngleAxisd angle_axis(q);
  tau.head<3>() = angle_axis.angle() * angle_axis.axis();

  return tau;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d product(Eigen::Affine3d const& T1, Eigen::Affine3d const& T2)
{
  Eigen::Affine3d result = T1;
  result.linear() = T1.rotation() * T2.rotation();
  result.translation() += T1.rotation() * T2.translation();
  return result;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d product(Eigen::Matrix<double,6,1> const& tau, Eigen::Affine3d const& T)
{
  return product(expMap(tau), T);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d inverse(Eigen::Affine3d const T)
{
  Eigen::Affine3d result = T;
  result.linear() = T.rotation().transpose();
  result.translation() = - T.translation();
  return result;
}

//--------------------------------------------------------------------------------------------------

} // namespace so3r3
} // namespace common
