#include <iostream>
#include <assert.h>

#include <vision/distortion_radtan.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

DistortionRadTan::DistortionRadTan(
  Eigen::VectorXd const& distortion_coeffs)
: DistortionModel(distortion_coeffs,DistortionModel::Type::RadTan)
{
  assert(this->distortion_coeffs_.size() == 5);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void DistortionRadTan::distort(Eigen::Vector2d* point, Eigen::Matrix2d* out_jacobian) const
{
  // Get the coefficients
  double const& k1 = this->distortion_coeffs_(0);
  double const& k2 = this->distortion_coeffs_(1);
  double const& p1 = this->distortion_coeffs_(2);
  double const& p2 = this->distortion_coeffs_(3);
  double const& k3 = this->distortion_coeffs_(4);
  
  // Compute the distortion components
  double& x = (*point)(0);
  double& y = (*point)(1);
  double const x2 = x*x;
  double const y2 = y*y;
  double const xy = x*y;
  double const r2 = x2 + y2;
  double const radial_dist = k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
  
  // Compute the Jacobian matrix if required
  if(out_jacobian)
  {
    double const dxd_dx = radial_dist + (2.*k1 + 4.*k2*r2 + 6.*k3*r2*r2)*x2 + (2.*p1*y + 6.*p2*x);
    double const dxd_dy = (2.*k1 + 4.*k2*r2)*xy + (2.*p1*x + 2.*p2*y);
    double const dyd_dx = dxd_dy;
    double const dyd_dy = radial_dist + (2.*k1 + 4.*k2*r2 + 6.*k3*r2*r2)*y2 + (2.*p2*x + 6.*p1*y);
    (*out_jacobian) << dxd_dx, dxd_dy,
                       dyd_dx, dyd_dy;
  }
  
  // Compute the distorted coordinates
  x += radial_dist*x + 2*p1*xy + p2*(r2+2*x2);
  y += radial_dist*y + 2*p2*xy + p1*(r2+2*y2);
}

//--------------------------------------------------------------------------------------------------

void DistortionRadTan::getDistortionCoeffs(cv::Mat* coeffs_ptr) const
{
  assert(coeffs_ptr != NULL);
  double const* it_begin = this->distortion_coeffs_.data();
  std::vector<double> data(it_begin, it_begin + 6);
  *coeffs_ptr = cv::Mat(6, 1, CV_64FC1, data.data());
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
