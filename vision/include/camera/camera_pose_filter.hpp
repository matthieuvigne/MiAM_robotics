#ifndef CAMERA_CAMERA_POSE_FILTER_HPP
#define CAMERA_CAMERA_POSE_FILTER_HPP

#include <eigen3/Eigen/Dense>

#include <common/macros.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraPoseFilter {

public:

  POINTER_TYPEDEF(CameraPoseFilter);

public:

  DISALLOW_EVIL_CONSTRUCTORS(CameraPoseFilter);
  CameraPoseFilter(
    Eigen::Affine3d const& T_WM,
    Eigen::Affine3d const& T_RC,
    Eigen::Matrix<double,6,6> const& cov_T_RC);
  virtual ~CameraPoseFilter(){}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  // Initialization

  inline void setState(Eigen::Affine3d const& T_WC);
  inline void setCovariance(Eigen::Matrix<double,6,6> const& cov_T_WC);

  enum class InitType { T_WC, T_CM };
  void setStateAndCovariance(
    InitType init_type,
    Eigen::Affine3d const& T,
    Eigen::Matrix<double,6,6> const cov_T);

  // Estimation

  void predict(
    double wy,
    double cov_wy);

  void update(
    Eigen::Affine3d const& T_CM,
    Eigen::Matrix<double,6,6> const& cov_T_CM);

  // Getters
  inline Eigen::Affine3d const& getState() const;
  inline Eigen::Matrix<double,6,6> const& getStateCovariance() const;

private:

  // Filter state and covariance
  Eigen::Affine3d T_WC_;
  Eigen::Matrix<double,6,6> cov_T_WC_;

  // Parameters
  Eigen::Affine3d const T_WM_;
  Eigen::Affine3d const T_RC_;
  Eigen::Matrix<double,6,6> cov_T_RC_;

}; // class CameraPoseFilter

//--------------------------------------------------------------------------------------------------
// Inline functions
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

} // namespace camera

#endif // CAMERA_CAMERA_POSE_FILTER
