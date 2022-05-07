#include <common/logger.hpp>
#include <camera/camera_pose_filter.hpp>

#define NUM_REQUIRED_UPDATES 10

namespace camera {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

CameraPoseFilter::CameraPoseFilter(Params const& params)
: team_                 (params.team),
  WpC_                  (initializeWpc(params.team)),
  azimuth_deg_          (0.00),
  cov_                  (initializeCovariance(params.sigma_position, params.sigma_azimuth_deg)),
  elevation_deg_        (45.0),
  sigma_elevation_deg_  (params.sigma_elevation_deg),
  qWR_                  (initializeQwr()),
  TWM_                  (Eigen::Translation3d(1.5,1.0,0.0)*Eigen::AngleAxisd()),
  is_initialized_       (false),
  num_updates_          (0)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Vector3d CameraPoseFilter::initializeWpc(common::Team team)
{
  Eigen::Vector3d WpC;
  switch(team)
  {
    case common::Team::UNKNOWN:
      WpC = Eigen::Vector3d{1.50,1.80,1.00};
      break;
    case common::Team::PURPLE:
      WpC = Eigen::Vector3d{1.60,1.80,1.00};
      break;
    case common::Team::YELLOW:
      WpC = Eigen::Vector3d{1.40,1.80,1.00};
      break;
    default:
      throw std::runtime_error("Unknown team");
  }
  return WpC;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix4d CameraPoseFilter::initializeCovariance(
  double sigma_position, double sigma_azimuth_deg)
{
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
  covariance(0,0) = std::pow(sigma_azimuth_deg, 2.0);
  covariance.block<3,3>(1,1) *= std::pow(sigma_position, 2.0);
  return covariance;
}

//--------------------------------------------------------------------------------------------------

Eigen::Quaterniond CameraPoseFilter::getQrc(double azimuth_deg, double elevation_deg)
{
  /* Sign conventions:
   * 500  = -90°  (looks left)
   * 1500 =   0°  (looks front)
   * 2500 = +90°  (looks right)
   */
  Eigen::Quaterniond qRC;
  qRC = Eigen::AngleAxisd(-azimuth_deg*RAD,   Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-elevation_deg*RAD, Eigen::Vector3d::UnitY());
  return qRC;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d CameraPoseFilter::getTWC() const
{
  Eigen::Quaterniond const qRC = getQrc(azimuth_deg_, elevation_deg_);
  Eigen::Affine3d TWC = Eigen::Translation3d(WpC_) * qWR_ * qRC;
  return TWC;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::predict(double azimuth_deg)
{
  // Propagate the covariance
  double const delta_azimuth_deg = azimuth_deg - azimuth_deg_;
  double constexpr sigma_azimuth_deg = 1.0;
  cov_(0,0) += std::pow(sigma_azimuth_deg, 2.0);

  // Update the pose
  azimuth_deg_ = azimuth_deg;
}

//--------------------------------------------------------------------------------------------------

CameraPoseFilter::Params CameraPoseFilter::Params::getDefaultParams(common::Team team)
{
  Params params;
  params.team = team;
  params.sigma_position = 5.0e-2;
  params.sigma_azimuth_deg = 5.0;
  params.sigma_elevation_deg = 5.0;
  return params;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::update(
  Eigen::Affine3d const& measured_TCM,
  Eigen::Matrix<double,6,6> const& cov_TCM)
{
  // Predict the measurement
  Eigen::Affine3d const TWC = getTWC();
  Eigen::Affine3d const TRC = Eigen::Translation3d() * getQrc(azimuth_deg_, elevation_deg_);
  Eigen::Affine3d const TWR = Eigen::Translation3d(WpC_) * qWR_;
  Eigen::Affine3d const predicted_TCM = TRC.inverse() * TWR.inverse() * TWM_;
  
  // Compute innovation within the Lie algebra
  // T_CM_mes = exp(tau)*T_CM_pred => tau = log(T_CM_mes*T_CM_pred^-1)
  Eigen::Affine3d const error_TCM = measured_TCM * predicted_TCM.inverse();
  Eigen::Matrix<double,6,1> innov = common::so3r3::logMap(error_TCM);
  innov.head<3>() *= RAD;

  // Compute the measure-state cross covariance matrices
  // Recall: the covariance is of the form [orientation (deg), position (m)]

  Eigen::Matrix<double,6,6> J_TCM_wrt_TCW = common::so3r3::leftSe3ProductJacobian(TWC.inverse(), TWM_);
  Eigen::Matrix<double,6,6> J_TCW_wrt_TWC = common::so3r3::se3InverseJacobian(TWC);
  Eigen::Matrix<double,6,3> J_TWC_wrt_WpC = Eigen::Matrix<double,6,6>::Identity().block<6,3>(0,3);
  Eigen::Matrix<double,6,3> J_TCM_wrt_WpC = J_TCM_wrt_TCW * J_TCW_wrt_TWC * J_TWC_wrt_WpC;

  Eigen::Affine3d const TRM = TWR.inverse() * TWM_;
  Eigen::Matrix<double,6,6> J_TCM_wrt_TCR = common::so3r3::leftSe3ProductJacobian(TRC.inverse(), TRM);
  Eigen::Matrix<double,6,6> J_TCR_wrt_TRC = common::so3r3::se3InverseJacobian(TRC);
  Eigen::Matrix<double,6,1> J_TRC_wrt_azimuth = - Eigen::Matrix<double,6,1>::Unit(0)*RAD;
  Eigen::Matrix<double,6,1> J_TCM_wrt_azimuth = J_TCM_wrt_TCR * J_TCR_wrt_TRC * J_TRC_wrt_azimuth;

  Eigen::Matrix<double,6,4> J_TCM_wrt_state;
  J_TCM_wrt_state << J_TCM_wrt_azimuth, J_TCM_wrt_WpC;

  // Update the state estimates
  // Recall: the measurement covariance should be of the form [orientation (deg), position (m)]
  Eigen::Matrix<double,6,6> S = J_TCM_wrt_state * cov_ * J_TCM_wrt_state.transpose() + cov_TCM;
  Eigen::Matrix<double,4,6> K = cov_ * J_TCM_wrt_state.transpose() * S.inverse();
  Eigen::Vector4d new_state = K * innov;
  azimuth_deg_ = new_state(0);
  WpC_ = new_state.tail<3>();
  Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
  cov_ = (I4 - K*J_TCM_wrt_state)*cov_;
  
  // Decide whether the filter is initialized
  if(!is_initialized_ && (++num_updates_==NUM_REQUIRED_UPDATES))
  {
    is_initialized_ = true;
  }
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> CameraPoseFilter::getCovTWC() const
{
  Eigen::Matrix<double,6,3> J_TWC_wrt_WpC = Eigen::Matrix<double,6,6>::Identity().block<6,3>(0,3);
  
  Eigen::Affine3d TWR = Eigen::Translation3d(WpC_) * qWR_;
  Eigen::Affine3d TRC = Eigen::Translation3d() * getQrc(azimuth_deg_, elevation_deg_);
  Eigen::Matrix<double,6,6> J_TWC_wrt_TRC = common::so3r3::rightSe3ProductJacobian(TWR, TRC);
  Eigen::Matrix<double,6,1> J_TRC_wrt_azimuth = - Eigen::Matrix<double,6,1>::Unit(0)*RAD;
  Eigen::Matrix<double,6,1> J_TWC_wrt_azimuth = J_TWC_wrt_TRC * J_TRC_wrt_azimuth;
  
  Eigen::Matrix<double,6,4> J_TWC_wrt_state;
  J_TWC_wrt_state << J_TWC_wrt_azimuth, J_TWC_wrt_WpC;
  
  Eigen::Matrix<double,6,6> covariance = J_TWC_wrt_state * cov_ * J_TWC_wrt_state.transpose();
  return covariance;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
