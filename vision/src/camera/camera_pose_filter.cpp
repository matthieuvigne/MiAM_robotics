#include <common/common.hpp>
#include <common/logger.hpp>
#include <camera/camera_pose_filter.hpp>

#define NUM_REQUIRED_UPDATES 5

namespace camera {

// Mauvais repère, pas la même taille pour le 42

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

CameraPoseFilter::CameraPoseFilter(Params const& params)
: team_                 (params.team),
  WpR_                  (common::getWpRi(params.team)),
  azimuth_deg_          (0.00),
  cov_                  (initializeCovariance(params.sigma_position, 
                          params.sigma_azimuth_deg, params.sigma_elevation_deg)),
  elevation_deg_        (45.0),
  qWR_                  (common::getqWR()),
  TWM_                  (common::getTWM()),
  is_initialized_       (false),
  num_updates_          (0)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,5,5> CameraPoseFilter::initializeCovariance(
  double sigma_position, double sigma_azimuth_deg, double sigma_elevation_deg)
{
  Eigen::Matrix<double,5,5> covariance = Eigen::Matrix<double,5,5>::Identity();
  covariance(0,0) = std::pow(sigma_azimuth_deg, 2.0);
  covariance(1,1) = std::pow(sigma_elevation_deg, 2.0);
  covariance.block<3,3>(2,2) *= std::pow(sigma_position, 2.0);
  return covariance;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d CameraPoseFilter::getTWC() const
{
  Eigen::Affine3d const TRC = common::getTRC(azimuth_deg_, elevation_deg_);
  Eigen::Affine3d const TWR = Eigen::Translation3d(WpR_) * qWR_;
  return Eigen::Affine3d(TWR*TRC);
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::predict(double delta_azimuth_deg)
{
  azimuth_deg_ += delta_azimuth_deg;
  double constexpr sigma_azimuth_deg = 1.0;
  cov_(0,0) += std::pow(sigma_azimuth_deg, 2.0);
}

//--------------------------------------------------------------------------------------------------

CameraPoseFilter::Params CameraPoseFilter::Params::getDefaultParams(common::Team team)
{
  Params params;
  params.team = team;
  params.sigma_position = 5.0e-2;
  params.sigma_azimuth_deg = 5.0;
  params.sigma_elevation_deg = 2.0;
  return params;
}

//--------------------------------------------------------------------------------------------------

void CameraPoseFilter::update(
  Eigen::Affine3d const& measured_TCM,
  Eigen::Matrix<double,6,6> const& cov_TCM)
{
  // Predict the measurement
  Eigen::Affine3d const TWC = getTWC();
  Eigen::Matrix<double,6,1> J_TRC_wrt_azimuth, J_TRC_wrt_elevation;
  Eigen::Affine3d const TRC = common::getTRC(azimuth_deg_, elevation_deg_,
    &J_TRC_wrt_azimuth, &J_TRC_wrt_elevation);
  Eigen::Affine3d const TWR = Eigen::Translation3d(WpR_) * qWR_;
  Eigen::Affine3d const predicted_TCM = TRC.inverse() * TWR.inverse() * TWM_;

  // Compute innovation within the Lie algebra
  Eigen::Affine3d const error_TCM = measured_TCM * predicted_TCM.inverse();
  innov_ = common::so3r3::logMap(error_TCM);
  innov_.head<3>() *= DEG;

  // Compute the measure-state cross covariance matrices
  // Recall: the covariance is of the form [orientation (deg), position (m)]

  Eigen::Matrix<double,6,6> J_TCM_wrt_TCW = common::so3r3::leftSe3ProductJacobian(TWC.inverse(), TWM_);
  Eigen::Matrix<double,6,6> J_TCW_wrt_TWC = common::so3r3::se3InverseJacobian(TWC);
  Eigen::Matrix<double,6,6> J_TWC_wrt_TWR = common::so3r3::leftSe3ProductJacobian(TWR, TRC);
  Eigen::Matrix<double,6,3> J_TWR_wrt_WpR = Eigen::Matrix<double,6,6>::Identity().block<6,3>(0,3);
  Eigen::Matrix<double,6,3> J_TCM_wrt_WpR = J_TCM_wrt_TCW * J_TCW_wrt_TWC * J_TWC_wrt_TWR * J_TWR_wrt_WpR;

  Eigen::Affine3d const TRM = TWR.inverse() * TWM_;
  Eigen::Matrix<double,6,6> J_TCM_wrt_TCR = common::so3r3::leftSe3ProductJacobian(TRC.inverse(), TRM);
  Eigen::Matrix<double,6,6> J_TCR_wrt_TRC = common::so3r3::se3InverseJacobian(TRC);
  Eigen::Matrix<double,6,1> J_TCM_wrt_azimuth = J_TCM_wrt_TCR * J_TCR_wrt_TRC * J_TRC_wrt_azimuth;
  Eigen::Matrix<double,6,1> J_TCM_wrt_elevation = J_TCM_wrt_TCR * J_TCR_wrt_TRC * J_TRC_wrt_elevation;

  Eigen::Matrix<double,6,5> J_TCM_wrt_state;
  J_TCM_wrt_state << J_TCM_wrt_azimuth, J_TCM_wrt_elevation, J_TCM_wrt_WpR;

  // Update the state estimates
  // Recall: the measurement covariance should be of the form [orientation (deg), position (m)]
  Eigen::Matrix<double,6,6> S = J_TCM_wrt_state * cov_ * J_TCM_wrt_state.transpose() + cov_TCM;
  Eigen::Matrix<double,5,6> K = cov_ * J_TCM_wrt_state.transpose() * S.inverse();
  azimuth_deg_ += K.row(0) * innov_;
  elevation_deg_ += K.row(1) * innov_;
  WpR_ += K.bottomRows(3) * innov_;
  cov_ -= K*J_TCM_wrt_state*cov_;

  // Decide whether the filter is initialized
  if(!is_initialized_ && (++num_updates_==NUM_REQUIRED_UPDATES))
  {
    is_initialized_ = true;
  }
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> CameraPoseFilter::getCovTWC() const
{
  // Reference and camera poses
  Eigen::Affine3d TWR = Eigen::Translation3d(WpR_) * qWR_;
  Eigen::Matrix<double,6,1> J_TRC_wrt_azimuth, J_TRC_wrt_elevation;
  Eigen::Affine3d TRC = common::getTRC(azimuth_deg_, elevation_deg_,
    &J_TRC_wrt_azimuth, &J_TRC_wrt_elevation);

  // Jacobian matrix wrt reference position
  Eigen::Matrix<double,6,6> J_TWC_wrt_TWR = common::so3r3::leftSe3ProductJacobian(TWR, TRC);
  Eigen::Matrix<double,6,3> J_TWR_wrt_WpR = Eigen::Matrix<double,6,6>::Identity().block<6,3>(0,3);
  Eigen::Matrix<double,6,3> J_TWC_wrt_WpR = J_TWC_wrt_TWR * J_TWR_wrt_WpR;

  // Jacobian matrix wrt azimuth angle
  Eigen::Matrix<double,6,6> J_TWC_wrt_TRC = common::so3r3::rightSe3ProductJacobian(TWR, TRC);
  Eigen::Matrix<double,6,1> J_TWC_wrt_azimuth = J_TWC_wrt_TRC * J_TRC_wrt_azimuth;
  Eigen::Matrix<double,6,1> J_TWC_wrt_elevation = J_TWC_wrt_TRC * J_TRC_wrt_elevation;

  // Complete Jacobian matrix
  Eigen::Matrix<double,6,5> J_TWC_wrt_state;
  J_TWC_wrt_state << J_TWC_wrt_azimuth, J_TWC_wrt_elevation, J_TWC_wrt_WpR;
  Eigen::Matrix<double,6,6> covariance = J_TWC_wrt_state * cov_ * J_TWC_wrt_state.transpose();
  return covariance;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
