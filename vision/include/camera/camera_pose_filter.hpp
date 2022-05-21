#ifndef CAMERA_CAMERA_POSE_FILTER_HPP
#define CAMERA_CAMERA_POSE_FILTER_HPP

#include <eigen3/Eigen/Dense>

#include <common/common.hpp>
#include <common/macros.hpp>
#include <common/maths.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraPoseFilter {

public:

  POINTER_TYPEDEF(CameraPoseFilter);
  struct Params {
    common::Team team;
    double sigma_position;
    double sigma_azimuth_deg;
    double sigma_elevation_deg;
    static Params getDefaultParams(common::Team team);
  }; // struct Params

public:

  DISALLOW_EVIL_CONSTRUCTORS(CameraPoseFilter);
  CameraPoseFilter(Params const& params);
  virtual ~CameraPoseFilter(){}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  // Estimation
  void predict(double delta_azimuth_deg);
  void update(Eigen::Affine3d const& TCM, Eigen::Matrix<double,6,6> const& covTCM);

  // Getters
  Eigen::Affine3d getTWC() const;
  Eigen::Matrix<double,6,6> getCovTWC() const;
  inline Eigen::Vector3d getWpR() const { return WpR_; }
  inline double getAzimuthDeg() const { return azimuth_deg_; }
  inline double getElevationDeg() const { return elevation_deg_; }
  inline Eigen::Matrix<double,6,1> getLastInnovation() const { return innov_; }
  inline bool isInitialized() const { return is_initialized_; }
  inline common::Team getTeam() const;

private:

  static Eigen::Matrix<double,5,5> initializeCovariance(
    double sigma_pos,
    double sigma_azimuth_deg,
    double sigma_elevation_deg);

private:

  // Team
  common::Team const team_;

  // Filter state and covariance
  Eigen::Vector3d WpR_;
  double azimuth_deg_;
  double elevation_deg_;
  Eigen::Matrix<double,5,5> cov_; ///< [azimuth (deg), elevation (deg), position (m)]
  Eigen::Matrix<double,6,1> innov_;

  // Parameters
  Eigen::Quaterniond const qWR_;
  Eigen::Affine3d const TWM_;
  bool is_initialized_;
  int num_updates_;

}; // class CameraPoseFilter

//--------------------------------------------------------------------------------------------------

common::Team CameraPoseFilter::getTeam() const
{
  return team_;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_POSE_FILTER
