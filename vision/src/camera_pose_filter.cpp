#include <vision/camera_pose_filter.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

CameraPoseFilter::CameraPoseFilter(
  Eigen::Affine3d const& T_RC,
  Eigen::Matrix<double,6,6> const& cov_T_RC)
: T_RC_ (T_RC),
  cov_T_RC_ (cov_T_RC)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d const& CameraPoseFilter::getState() const
{
  return this->T_WC_;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> const& CameraPoseFilter::getStateCovariance() const
{
  return this->cov_T_WC_;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::setState(
  Eigen::Affine3d const& T_WC)
{
  // Set the position
  this->T_WC_ = T_WC;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::setCovariance(
  Eigen::Matrix<double,6,6> const& cov_T_WC)
{
  this->cov_T_WC_ = cov_T_WC;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::predict(
  double dthetaz_rad,
  double cov_dthetaz_rad)
{
  // Prediction of the state
  Eigen::Affine3d const T_Rk_Rkp1(Eigen::AngleAxisd(dthetaz_rad,Eigen::Vector3d::UnitZ()));
  this->T_WC_ = this->T_WC_ * this->T_RC_.inverse() * T_Rk_Rkp1 * this->T_RC_;
  
  // Computation of the Jacobian matrices
  Eigen::Matrix<double,6,6> J_TWCkp1_wrt_TWCk;
  Eigen::Matrix<double,6,6> J_TWCkp1_wrt_TRC;
  Eigen::Matrix<double,6,1> J_TWCkp1_wrt_dthetaz;
  // Compute ...
  
  // Prediction of the covariance
  this->cov_T_WC_ = 
    J_TWCkp1_wrt_TWCk * this->cov_T_WC_ * J_TWCkp1_wrt_TWCk.transpose() +
    J_TWCkp1_wrt_TRC * this->cov_T_RC_ * J_TWCkp1_wrt_TRC.transpose() +
    J_TWCkp1_wrt_dthetaz * cov_dthetaz_rad * J_TWCkp1_wrt_dthetaz.transpose();
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> CameraPoseFilter::leftProductJacobian(
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

Eigen::Matrix<double,6,6> CameraPoseFilter::rightProductJacobian(
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

Eigen::Matrix<double,6,6> CameraPoseFilter::inverseJacobian(
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

Eigen::Matrix3d CameraPoseFilter::skew(Eigen::Vector3d const& w)
{
  Eigen::Matrix3d S;
  S <<    0., -w.z(),  w.y(),
       w.z(),     0., -w.x(),
      -w.y(),  w.x(),     0.;
  return S;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d CameraPoseFilter::leftJacobianSO3(Eigen::Vector3d const& theta)
{
  double angle = theta.norm();
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  if (angle==0) return J;
  Eigen::Matrix3d const S = skew(theta);
  J += ((1-std::cos(angle))/angle)*S + ((angle-std::sin(angle))/std::pow(angle,3))*S*S;
  return J;
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
