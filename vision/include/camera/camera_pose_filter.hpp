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
  CameraPoseFilter(
    Eigen::Affine3d const& T_WM,
    Eigen::Affine3d const& T_RC,
    double sigma_RRC, double sigma_RtC);
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
    Eigen::Vector3d const& w,
    Eigen::Matrix3d const& cov_w);
  enum Axis {X, Y, Z};
  void predict(double wi, double cov_wi, Axis axis);

  void update(
    Eigen::Affine3d const& T_CM,
    Eigen::Matrix<double,6,6> const& cov_T_CM);

  // Getters

  inline Eigen::Affine3d const& getState() const;
  inline Eigen::Matrix<double,6,6> const& getStateCovariance() const;
  inline Eigen::Affine3d const& getTRC() const;
  inline Eigen::Matrix<double,6,6> const& getCovTRC() const;

  // Print & checks
  
  std::string printEstimateAndCovariance() const;
  bool isCovarianceMatrixIsSymmetric() const;

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

Eigen::Affine3d const& CameraPoseFilter::getTRC() const
{
  return T_RC_;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> const& CameraPoseFilter::getCovTRC() const
{
  return cov_T_RC_;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_POSE_FILTER
