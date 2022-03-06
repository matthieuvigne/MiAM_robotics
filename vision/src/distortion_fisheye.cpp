#include <iostream>
#include <assert.h>

#include <vision/distortion_fisheye.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

DistortionFisheye::DistortionFisheye(
  Eigen::VectorXd const& distortion_coeffs)
: DistortionModel(distortion_coeffs,DistortionModel::Type::RadTan)
{
  assert(this->distortion_coeffs_.size() == 4);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void DistortionFisheye::distort(Eigen::Vector2d* point, Eigen::Matrix2d* out_jacobian) const
{
  // Get the coefficients
  double const& k1 = this->distortion_coeffs_(0);
  double const& k2 = this->distortion_coeffs_(1);
  double const& k3 = this->distortion_coeffs_(2);
  double const& k4 = this->distortion_coeffs_(3);
  
  // Compute the distortion components
  double& x = (*point)(0);
  double& y = (*point)(1);
  double const x2 = x*x;
  double const y2 = y*y;
  double const r2 = x2 + y2;
  double const theta = std::atan(std::sqrt(r2));
  double const theta2 = theta*theta;
  double const theta4 = theta2*theta2;
  double const theta6 = theta4*theta2;
  double const theta8 = theta4*theta4;
  double const rad_dist = 1 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8;
  double const full_dist = (theta/r)*rad_dist;
  
  // Compute the Jacobian matrix if required
  if(out_jacobian)
  {
    // Get derivatives of the radius
    double const dr2_dx = 2*x;
    double const dr2_dy = 2*y;
    double const dtheta_dr = 1./(1.+r2);
    double const dr_dx = x/r;
    double const dr_dy = y/r;
    double const dtheta_dx = dtheta_dr * dr_dx;
    double const dtheta_dy = dtheta_dr * dr_dy;

    // Get derivatives of the distortion factor
    double const draddist_dr = 
      (2*k1*theta + 4*k2*theta2*theta + 6*theta4*theta + 8*k4*theta6*theta)*dtheta_dr;
    double const dfulldist_dr =
      (dtheta_dr/r-theta/r2)*rad_dist + (theta/r)*draddist_dr;
    double const dfulldist_dx = dfulldist_dy*dr_dx;
    double const dfulldist_dy = dfulldist_dr*dr_dy;
    
    // Compute the full gradient
    double const dxd_dx = full_dist*1.0 + dfulldist_dx*x;
    double const dxd_dy = dfulldist_dy*x;
    double const dyd_dx = dfulldist_dx*y;
    double const dyd_dy = full_dist*1.0 + dfulldist_dy*y;
    (*out_jacobian) << dxd_dx, dxd_dy,
                       dyd_dx, dyd_dy;
  }
  
  // Compute the distorted coordinates
  x *= full_dist;
  y *= full_dist;
}

//--------------------------------------------------------------------------------------------------

void DistortionFisheye::getDistortionCoeffs(cv::Mat* coeffs_ptr) const
{
  assert(coeffs_ptr != NULL);
  double const* it_begin = this->distortion_coeffs_.data();
  std::vector<double> data(it_begin, it_begin + 4);
  *coeffs_ptr = cv::Mat(4, 1, CV_64FC1, data.data());
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
