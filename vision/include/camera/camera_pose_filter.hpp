#ifndef CAMERA_CAMERA_POSE_FILTER_HPP
#define CAMERA_CAMERA_POSE_FILTER_HPP

#include <eigen3/Eigen/Dense>

#include <common/common.hpp>
#include <common/macros.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class CameraPoseFilter {

public:

  POINTER_TYPEDEF(CameraPoseFilter);
  struct Params {
    common::Team team;
    Eigen::Affine3d T_WC;
    Eigen::Affine3d T_RC;
    Eigen::Matrix<double,6,6> cov_TRC;
    Eigen::Matrix<double,6,6> cov_TWC;
    static Params getDefaultParams(common::Team team);
  }; // struct Params

public:

  DISALLOW_EVIL_CONSTRUCTORS(CameraPoseFilter);
  CameraPoseFilter(Params const& params);
  virtual ~CameraPoseFilter(){}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  // Initialization

  enum class InitType { T_WC, T_CM };
  void setStateAndCovariance(
    InitType init_type,
    Eigen::Affine3d const& T,
    Eigen::Matrix<double,6,6> const cov_T);

  // Estimation

  void predict(
    Eigen::Vector3d const& w,
    Eigen::Matrix3d const& cov_w);
  enum class Axis {X, Y, Z};
  void predict(double wi, double cov_wi, Axis axis);
  void predict(double wrx_rad, double sigma_wrx_rad,
               double wry_rad, double sigma_wry_rad,
               double wrz_rad, double sigma_wrz_rad); // -> simplify

  void update(
    Eigen::Affine3d const& T_CM,
    Eigen::Matrix<double,6,6> const& cov_T_CM);

  // Getters

  inline Eigen::Affine3d const& getState() const;
  inline Eigen::Matrix<double,6,6> const& getStateCovariance() const;
  inline Eigen::Affine3d const& getTRC() const;
  inline Eigen::Matrix<double,6,6> const& getCovTRC() const;
  inline Eigen::Affine3d const& getTWM() const;

  // Print & checks
  
  inline bool isInitialized() const { return is_initialized_; }
  std::string printEstimateAndCovariance() const;
  bool isCovarianceMatrixIsSymmetric() const;

private:

  // Team
  common::Team team_;

  // Filter state and covariance
  Eigen::Affine3d T_WC_;
  Eigen::Matrix<double,6,6> cov_T_WC_;
  bool is_initialized_ = false;

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
  return T_WC_;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> const& CameraPoseFilter::getStateCovariance() const
{
  return cov_T_WC_;
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

Eigen::Affine3d const& CameraPoseFilter::getTWM() const
{
  return T_WM_;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // CAMERA_CAMERA_POSE_FILTER
