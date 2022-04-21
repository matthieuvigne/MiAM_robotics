#include <common/maths.hpp>
#include <camera/camera_pose_filter.hpp>

namespace camera {

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

CameraPoseFilter::CameraPoseFilter(
  Eigen::Affine3d const& T_WM,
  Eigen::Affine3d const& T_RC,
  double sigma_RRC, double sigma_RtC)
: CameraPoseFilter(T_WM, T_RC, Eigen::Matrix<double,6,6>::Identity())
{
  cov_T_RC_.block<3,3>(0,0) *= std::pow(sigma_RRC,2.0);
  cov_T_RC_.block<3,3>(3,3) *= std::pow(sigma_RtC,2.0);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::setStateAndCovariance(
  InitType init_type,
  Eigen::Affine3d const& T,
  Eigen::Matrix<double,6,6> const cov_T)
{
  switch(init_type)
  {

    case InitType::T_WC:
    {
      // Initialize the pose
      Eigen::Affine3d const& T_WC = T;
      this->T_WC_ = T_WC;
    
      // Initialize the covariance matrix
      Eigen::Matrix<double,6,6> const& cov_T_WC = cov_T;
      this->cov_T_WC_ = cov_T_WC;
      break;
    }

    case InitType::T_CM:
    {
      // Initialize the pose
      Eigen::Affine3d const& T_CM = T;
      Eigen::Affine3d const T_MC = T_CM.inverse();
      this->T_WC_ = this->T_WM_ * T_MC;
      
      // Initialize the covariance matrix
      Eigen::Matrix<double,6,6> const& cov_T_CM = cov_T;
      Eigen::Matrix<double,6,6> const J_TWC_wrt_TMC = common::so3r3::rightSe3ProductJacobian(this->T_WM_, T_MC);
      Eigen::Matrix<double,6,6> const J_TMC_wrt_TCM = common::so3r3::se3InverseJacobian(T_CM);
      Eigen::Matrix<double,6,6> const J_TWC_wrt_TCM = J_TWC_wrt_TMC * J_TMC_wrt_TCM;
      this->cov_T_WC_ = J_TWC_wrt_TCM * cov_T_CM * J_TWC_wrt_TCM.transpose();
      this->cov_T_WC_ *= 1.25;
      break;
    }
  }
}

//--------------------------------------------------------------------------------------------------

///> Could still be optimized 
void CameraPoseFilter::predict(
  double dtheta_rad,
  double cov_dtheta_rad,
  Axis rotation_axis)
{
  Eigen::Vector3d rotation_vector_rad;
  Eigen::Matrix3d rotation_covariance_rad = Eigen::Matrix3d::Zero();
  switch(rotation_axis)
  {
    case Axis::X:
      rotation_vector_rad = dtheta_rad * Eigen::Vector3d::UnitX();
      rotation_covariance_rad(0,0) = cov_dtheta_rad;
      break;
    case Axis::Y:
      rotation_vector_rad = dtheta_rad * Eigen::Vector3d::UnitY();
      rotation_covariance_rad(1,1) = cov_dtheta_rad;
      break;
    case Axis::Z:
      rotation_vector_rad = dtheta_rad * Eigen::Vector3d::UnitZ();
      rotation_covariance_rad(2,2) = cov_dtheta_rad;
      break;
    default:
      throw std::runtime_error("Unknown rotation axis");
  }
  return this->predict(rotation_vector_rad, rotation_covariance_rad);
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::predict(
  Eigen::Vector3d const& dtheta_rad,
  Eigen::Matrix3d const& cov_dtheta_rad)
{
  // Computation of the intermediary poses
  
  Eigen::Affine3d const T_CR = this->T_RC_.inverse();
  Eigen::Affine3d const T_Rk_Rkp1 = common::so3r3::expMap(dtheta_rad, Eigen::Vector3d::Zero());
  Eigen::Affine3d const T_Ck_Ckp1 = T_CR * T_Rk_Rkp1 * this->T_RC_;
  Eigen::Affine3d const T_W_Ckp1 = this->T_WC_ * T_Ck_Ckp1;
  Eigen::Affine3d const T_W_Rk = this->T_WC_ * T_CR;
  Eigen::Affine3d const T_Rk_Ckp1 = T_Rk_Rkp1 * this->T_RC_;
  Eigen::Affine3d const T_W_Rkp1 = T_W_Rk * T_Rk_Rkp1;
  
  // Computation of the Jacobian matrices
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWCk =
    common::so3r3::leftSe3ProductJacobian(this->T_WC_, T_Ck_Ckp1);

  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWRk =
    common::so3r3::leftSe3ProductJacobian(T_W_Rk, T_Rk_Ckp1);
  Eigen::Matrix<double,6,6> const J_TWRk_wrt_T_CR =
    common::so3r3::rightSe3ProductJacobian(this->T_WC_, T_CR);
  Eigen::Matrix<double,6,6> const J_TCR_wrt_TRC =
    common::so3r3::se3InverseJacobian(this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRC =
    common::so3r3::rightSe3ProductJacobian(T_W_Rkp1, this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TRC =
    J_TWCkp1_wrt_TWRk * J_TWRk_wrt_T_CR * J_TCR_wrt_TRC + J_TWRkp1_wrt_TRC;
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_T_WRkp1 =
    common::so3r3::leftSe3ProductJacobian(T_W_Rkp1, this->T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRkRkp1 =
    common::so3r3::rightSe3ProductJacobian(T_W_Rk, T_Rk_Rkp1);
  Eigen::Matrix3d const Jtheta = common::leftJacobianSO3(dtheta_rad);
  Eigen::Matrix<double,6,3> J_TRkRkp1_wrt_dtheta = Eigen::Matrix<double,6,3>::Zero();
  J_TRkRkp1_wrt_dtheta.block<3,3>(0,0) = Jtheta;
  J_TRkRkp1_wrt_dtheta.block<3,3>(3,0) = common::skew(this->T_WC_.translation()) * Jtheta;
  Eigen::Matrix<double,6,3> const J_TWCkp1_wrt_dtheta =
    J_TWCkp1_wrt_T_WRkp1 * J_TWRkp1_wrt_TRkRkp1 * J_TRkRkp1_wrt_dtheta;
  
  // Prediction of the state and the covariance
  this->cov_T_WC_ = 
    J_TWCkp1_wrt_TWCk * this->cov_T_WC_ * J_TWCkp1_wrt_TWCk.transpose() +
    J_TWCkp1_wrt_TRC * this->cov_T_RC_ * J_TWCkp1_wrt_TRC.transpose() +
    J_TWCkp1_wrt_dtheta * cov_dtheta_rad * J_TWCkp1_wrt_dtheta.transpose();
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
  Eigen::Matrix<double,6,1> const innov = common::so3r3::logMap(T_CM*T_CM_pred.inverse());
  
  // Compute the innovation covariance
  Eigen::Affine3d const T_CW = this->T_WC_.inverse();
  Eigen::Matrix<double,6,6> const J_TCM_wrt_TCW = common::so3r3::leftSe3ProductJacobian(T_CW, this->T_WM_);
  Eigen::Matrix<double,6,6> const J_TCW_wrt_TWC = common::so3r3::se3InverseJacobian(this->T_WC_);
  Eigen::Matrix<double,6,6> const J_TCM_wrt_TWC = J_TCM_wrt_TCW * J_TCW_wrt_TWC;
  Eigen::Matrix<double,6,6> const S =
    J_TCM_wrt_TWC * this->cov_T_WC_ * J_TCM_wrt_TWC.transpose() + cov_T_CM;

  // Compute the gain matrix
  Eigen::Matrix<double,6,6> const K = this->cov_T_WC_ * J_TCM_wrt_TWC * S.inverse();
  
  // Update the state estimate
  this->T_WC_ = common::so3r3::expMap(K*innov) * this->T_WC_;
  
  // Update the state covariance matrix
  Eigen::Matrix<double,6,6> const I = Eigen::Matrix<double,6,6>::Identity();
  this->cov_T_WC_ = (I - K * J_TCM_wrt_TWC) * this->cov_T_WC_;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
