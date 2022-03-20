#include <vision/camera_pose_filter.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

CameraPoseFilter::CameraPoseFilter(
  Eigen::Affine3d const& T_WM,
  Eigen::Affine3d const& T_RC,
  Eigen::Matrix<double,6,6> const& cov_T_RC)
: T_WM_ (T_WM),
  T_RC_ (T_RC),
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

///> Could still be optimized 
void CameraPoseFilter::predict(
  double dthetay_rad,
  double cov_dthetay_rad)
{
  // Computation of the intermediary poses
  
  Eigen::Affine3d const T_CR = this->T_RC_.inverse();
  Eigen::Affine3d const T_Rk_Rkp1(Eigen::AngleAxisd(dthetay_rad,Eigen::Vector3d::UnitY()));
  Eigen::Affine3d const T_Ck_Ckp1 = T_CR * T_Rk_Rkp1 * this->T_RC_;
  Eigen::Affine3d const T_W_Ckp1 = this->T_WC_ * T_Ck_Ckp1;
  Eigen::Affine3d const T_W_Rk = this->T_WC_ * T_CR;
  Eigen::Affine3d const T_Rk_Ckp1 = T_Rk_Rkp1 * this->T_RC_;
  Eigen::Affine3d const T_W_Rkp1 = T_W_Rk * T_Rk_Rkp1;
  
  // Computation of the Jacobian matrices
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWCk = leftProductJacobian(this->T_WC_, T_Ck_Ckp1);
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWRk = leftProductJacobian(T_W_Rk, T_Rk_Ckp1);
  Eigen::Matrix<double,6,6> const J_TWRk_wrt_T_CR = rightProductJacobian(this->T_WC_, T_CR);
  Eigen::Matrix<double,6,6> const J_TCR_wrt_TRC = inverseJacobian(this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRC = rightProductJacobian(T_W_Rkp1, this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TRC =
    J_TWCkp1_wrt_TWRk * J_TWRk_wrt_T_CR * J_TCR_wrt_TRC + J_TWRkp1_wrt_TRC;
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_T_WRkp1 = leftProductJacobian(T_W_Rkp1, this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRkRkp1 = rightProductJacobian(T_W_Rk, T_Rk_Rkp1);
  Eigen::Vector3d const Jthetay = leftJacobianSO3(dthetay_rad * Eigen::Vector3d::UnitY()).col(2);
  Eigen::Matrix<double,6,1> J_TRkRkp1_wrt_dthetay = Eigen::Matrix<double,6,1>::Zero();
  J_TRkRkp1_wrt_dthetay.head<3>() = Jthetay;
  J_TRkRkp1_wrt_dthetay.tail<3>() = skew(this->T_WC_.translation()) * Jthetay;
  Eigen::Matrix<double,6,1> const J_TWCkp1_wrt_dthetay =
    J_TWCkp1_wrt_T_WRkp1 * J_TWRkp1_wrt_TRkRkp1 * J_TRkRkp1_wrt_dthetay;
  
  // Prediction of the state and the covariance
  this->cov_T_WC_ = 
    J_TWCkp1_wrt_TWCk * this->cov_T_WC_ * J_TWCkp1_wrt_TWCk.transpose() +
    J_TWCkp1_wrt_TRC * this->cov_T_RC_ * J_TWCkp1_wrt_TRC.transpose() +
    J_TWCkp1_wrt_dthetay * cov_dthetay_rad * J_TWCkp1_wrt_dthetay.transpose();
  this->T_WC_ = T_W_Ckp1;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::update(
  Eigen::Affine3d const& T_CM,
  Eigen::Matrix<double,6,6> const& cov_T_CM)
{
  // Predict the measurement
  Eigen::Affine3d const T_CM_pred = this->T_WC_.inverse() * this->T_WM_;
  
  // Compute innovation in Lie algebra
  // T_CM_mes = exp(tau)*T_CM_pred => tau = log(T_CM_mes*T_CM_pred^-1)
  Eigen::Matrix<double,6,1> const innov = logMap(T_CM*T_CM_pred.inverse());
  
  // Compute the innovation covariance
  Eigen::Affine3d const T_CW = this->T_WC_.inverse();
  Eigen::Matrix<double,6,6> const J_TCM_wrt_TCW = leftProductJacobian(T_CW, this->T_WM_);
  Eigen::Matrix<double,6,6> const J_TCW_wrt_TWC = inverseJacobian(this->T_WC_);
  Eigen::Matrix<double,6,6> const J_TCM_wrt_TWC = J_TCM_wrt_TCW * J_TCW_wrt_TWC;
  Eigen::Matrix<double,6,6> const S =
    J_TCM_wrt_TWC * this->cov_T_WC_ * J_TCM_wrt_TWC.transpose() + cov_T_CM;

  // Compute the gain matrix
  Eigen::Matrix<double,6,6> const K = this->cov_T_WC_ * J_TCM_wrt_TWC * S.inverse();
  
  // Update the state estimate
  this->T_WC_ = expMap(K*innov) * this->T_WC_;
  
  // Update the state covariance matrix
  Eigen::Matrix<double,6,6> const I = Eigen::Matrix<double,6,6>::Identity();
  this->cov_T_WC_ = (I - K * J_TCM_wrt_TWC) * this->cov_T_WC_;
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

Eigen::Affine3d CameraPoseFilter::expMap(Eigen::Matrix<double,6,1> const& tau)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Vector3d const axis = tau.head<3>().normalized();
  double const angle = tau.head<3>().norm();
  Eigen::AngleAxisd const angle_axis(angle,axis);
  T.rotate(angle_axis);
  T.translation() = tau.tail<3>();
  return T;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,1> CameraPoseFilter::logMap(Eigen::Affine3d const& T)
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

} // namespace vision
