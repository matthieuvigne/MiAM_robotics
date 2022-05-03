#include <common/maths.hpp>
#include <camera/camera_pose_filter.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

CameraPoseFilter::CameraPoseFilter(Params const& params)
: team_     (params.team),
  T_WC_     (params.T_WC),
  cov_T_WC_ (params.cov_TWC),
  T_WM_     (Eigen::Translation3d(1.5,1.0,0.0) * Eigen::AngleAxisd()),
  T_RC_     (params.T_RC),
  cov_T_RC_ (params.cov_TRC)
{}

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

CameraPoseFilter::Params CameraPoseFilter::Params::getDefaultParams(Team team)
{
  Params params;
  
  // Team information
  params.team = team;
  
  // Pose of the camera wrt. the reference frame
  params.T_RC = Eigen::Translation3d() * Eigen::AngleAxisd(-M_PI_4,Eigen::Vector3d::UnitX());
  params.cov_TRC.setIdentity();
  params.cov_TRC.block<3,3>(0,0) *= std::pow(3.0*RAD,2.0);  // Orientation uncertainty
  params.cov_TRC.block<3,3>(3,3) *= std::pow(1e-3,2.0);     // Position uncertainty
  
  // Pose of the camera wrt. the global frame
  switch(params.team)
  {
    case Team::UNKNOWN:
      params.T_WC = Eigen::Translation3d(1.50,1.80,1.00)
        * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(-3*M_PI_4,Eigen::Vector3d::UnitX());
      break;
    case Team::PURPLE:
      params.T_WC = Eigen::Translation3d(1.60,1.80,1.00)
        * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(-3*M_PI_4,Eigen::Vector3d::UnitX());
      break;
    case Team::YELLOW:
      params.T_WC = Eigen::Translation3d(1.40,1.80,1.00)
        * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(-3*M_PI_4,Eigen::Vector3d::UnitX());
      break;
    default:
      throw std::runtime_error("Unknown team");
  }
  params.cov_TWC.setIdentity();
  params.cov_TWC.block<3,3>(0,0) *= std::pow(5.0*RAD,2.0);  // Orientation uncertainty
  params.cov_TWC.block<3,3>(3,3) *= std::pow(3e-2,2.0);     // Position uncertainty
  
  return params;
}

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
      T_WC_ = T_WC;
    
      // Initialize the covariance matrix
      Eigen::Matrix<double,6,6> const& cov_T_WC = cov_T;
      cov_T_WC_ = cov_T_WC;
      break;
    }

    case InitType::T_CM:
    {
      // Initialize the pose
      Eigen::Affine3d const& T_CM = T;
      Eigen::Affine3d const T_MC = T_CM.inverse();
      T_WC_ = T_WM_ * T_MC;
      
      // Initialize the covariance matrix
      Eigen::Matrix<double,6,6> const& cov_T_CM = cov_T;
      Eigen::Matrix<double,6,6> const J_TWC_wrt_TMC =
        common::so3r3::rightSe3ProductJacobian(T_WM_, T_MC);
      Eigen::Matrix<double,6,6> const J_TMC_wrt_TCM =
        common::so3r3::se3InverseJacobian(T_CM);
      Eigen::Matrix<double,6,6> const J_TWC_wrt_TCM =
        J_TWC_wrt_TMC * J_TMC_wrt_TCM;
      cov_T_WC_ = J_TWC_wrt_TCM * cov_T_CM * J_TWC_wrt_TCM.transpose();
      cov_T_WC_ *= 1.25;
      break;
    }
    
    default:
      throw std::runtime_error("Unknown initialization type");
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

void CameraPoseFilter::predict(double wrx_rad, double sigma_wrx_rad,
                               double wry_rad, double sigma_wry_rad,
                               double wrz_rad, double sigma_wrz_rad)
{
  Eigen::Vector3d const w{wrx_rad, wry_rad, wrz_rad};
  Eigen::Matrix3d const cov_w(Eigen::Vector3d(sigma_wrx_rad,sigma_wry_rad,sigma_wrz_rad)
    .cwiseAbs2().asDiagonal());
  return this->predict(w, cov_w);
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::predict(
  Eigen::Vector3d const& dtheta_rad,
  Eigen::Matrix3d const& cov_dtheta_rad)
{
  // Computation of the intermediary poses
  
  Eigen::Affine3d const T_CR = T_RC_.inverse();
  Eigen::Affine3d const T_Rk_Rkp1 = common::so3r3::expMap(dtheta_rad, Eigen::Vector3d::Zero());
  Eigen::Affine3d const T_Ck_Ckp1 = T_CR * T_Rk_Rkp1 * T_RC_;
  Eigen::Affine3d const T_W_Ckp1 = T_WC_ * T_Ck_Ckp1;
  Eigen::Affine3d const T_W_Rk = T_WC_ * T_CR;
  Eigen::Affine3d const T_Rk_Ckp1 = T_Rk_Rkp1 * T_RC_;
  Eigen::Affine3d const T_W_Rkp1 = T_W_Rk * T_Rk_Rkp1;
  
  // Computation of the Jacobian matrices
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWCk =
    common::so3r3::leftSe3ProductJacobian(T_WC_, T_Ck_Ckp1);

  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TWRk =
    common::so3r3::leftSe3ProductJacobian(T_W_Rk, T_Rk_Ckp1);
  Eigen::Matrix<double,6,6> const J_TWRk_wrt_T_CR =
    common::so3r3::rightSe3ProductJacobian(T_WC_, T_CR);
  Eigen::Matrix<double,6,6> const J_TCR_wrt_TRC =
    common::so3r3::se3InverseJacobian(T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRC =
    common::so3r3::rightSe3ProductJacobian(T_W_Rkp1, T_RC_);
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_TRC =
    J_TWCkp1_wrt_TWRk * J_TWRk_wrt_T_CR * J_TCR_wrt_TRC + J_TWRkp1_wrt_TRC;
  
  Eigen::Matrix<double,6,6> const J_TWCkp1_wrt_T_WRkp1 =
    common::so3r3::leftSe3ProductJacobian(T_W_Rkp1, T_RC_);
  Eigen::Matrix<double,6,6> const J_TWRkp1_wrt_TRkRkp1 =
    common::so3r3::rightSe3ProductJacobian(T_W_Rk, T_Rk_Rkp1);
  Eigen::Matrix3d const Jtheta = common::leftJacobianSO3(dtheta_rad);
  Eigen::Matrix<double,6,3> J_TRkRkp1_wrt_dtheta = Eigen::Matrix<double,6,3>::Zero();
  J_TRkRkp1_wrt_dtheta.block<3,3>(0,0) = Jtheta;
  J_TRkRkp1_wrt_dtheta.block<3,3>(3,0) = common::skew(T_WC_.translation()) * Jtheta;
  Eigen::Matrix<double,6,3> const J_TWCkp1_wrt_dtheta =
    J_TWCkp1_wrt_T_WRkp1 * J_TWRkp1_wrt_TRkRkp1 * J_TRkRkp1_wrt_dtheta;
  
  // Prediction of the state and the covariance
  cov_T_WC_ = J_TWCkp1_wrt_TWCk * cov_T_WC_ * J_TWCkp1_wrt_TWCk.transpose() +
              J_TWCkp1_wrt_TRC * cov_T_RC_ * J_TWCkp1_wrt_TRC.transpose() +
              J_TWCkp1_wrt_dtheta * cov_dtheta_rad * J_TWCkp1_wrt_dtheta.transpose();
  T_WC_ = T_W_Ckp1;
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
    J_TCM_wrt_TWC * cov_T_WC_ * J_TCM_wrt_TWC.transpose() + cov_T_CM;

  // Compute the gain matrix
  Eigen::Matrix<double,6,6> const K = cov_T_WC_ * J_TCM_wrt_TWC.transpose() * S.inverse();
  
  // Update the state estimate
  T_WC_ = common::so3r3::boxplus(K*innov, T_WC_);
  
  // Update the state covariance matrix
  Eigen::Matrix<double,6,6> const I = Eigen::Matrix<double,6,6>::Identity();
  cov_T_WC_ = (I - K * J_TCM_wrt_TWC) * cov_T_WC_;
}

//--------------------------------------------------------------------------------------------------

std::string CameraPoseFilter::printEstimateAndCovariance() const
{
  std::stringstream out;
  std::cout << "TWC_est:\n" << T_WC_.matrix() << std::endl;
  std::cout << "cov_TWC_est:\n" << cov_T_WC_.matrix() << std::endl;
  return out.str();
}

//--------------------------------------------------------------------------------------------------

bool CameraPoseFilter::isCovarianceMatrixIsSymmetric() const
{
  double constexpr epsilon = 1e-10;
  Eigen::Matrix<double,6,6> const diff = cov_T_WC_ - cov_T_WC_.transpose();
  return (std::fabs(diff.norm()) < epsilon);
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
