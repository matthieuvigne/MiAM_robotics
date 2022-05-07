#ifndef CAMERA_CAMERA_POSE_FILTER_2_HPP
#define CAMERA_CAMERA_POSE_FILTER_2_HPP

#include <eigen3/Eigen/Dense>

#include <common/common.hpp>
#include <common/macros.hpp>
#include <common/maths.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraPoseFilter2 {

public:

  POINTER_TYPEDEF(CameraPoseFilter2);
  struct Params {
    common::Team team;
    double sigma_position;
    double sigma_azimuth_deg;
    double sigma_elevation_deg;
    static Params getDefaultParams(common::Team team);
  }; // struct Params

public:

  DISALLOW_EVIL_CONSTRUCTORS(CameraPoseFilter2);
  CameraPoseFilter2(Params const& params);
  virtual ~CameraPoseFilter2(){}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  // Estimation
  void predict(double azimuth_deg);
  void update(Eigen::Affine3d const& TCM, Eigen::Matrix<double,6,6> const& covTCM);

  // Getters
  Eigen::Affine3d getTWC() const;
  inline Eigen::Vector3d getWpC() const { return WpC_; }
  inline double getAzimuthDeg() const { return azimuth_deg_; }
  inline bool isInitialized() const { return is_initialized_; }

private:

  inline static Eigen::Quaterniond initializeQwr();
  static Eigen::Matrix4d initializeCovariance(double sigma_pos, double sigma_azimuth_deg);
  static Eigen::Vector3d initializeWpc(common::Team team);
  static Eigen::Quaterniond getQrc(double azimuth_deg, double elevation_deg);

private:

  // Team
  common::Team team_;

  // Filter state and covariance
  Eigen::Vector3d WpC_;
  double azimuth_deg_;
  Eigen::Matrix4d cov_; ///< [elevation (deg), position (m)]
  
  // Parameters
  double const elevation_deg_;
  double const sigma_elevation_deg_;
  Eigen::Quaterniond const qWR_;
  Eigen::Affine3d const TWM_;
  bool is_initialized_;
  int num_updates_;

}; // class CameraPoseFilter2

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

Eigen::Quaterniond CameraPoseFilter2::initializeQwr()
{
  Eigen::Quaterniond const qWR =
      Eigen::AngleAxisd( M_PI_2, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY());
  return qWR;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_POSE_FILTER_2
