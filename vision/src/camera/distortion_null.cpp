#include <iostream>
#include <assert.h>

#include <camera/distortion_null.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

DistortionNull::DistortionNull()
: DistortionModel(Eigen::VectorXd(),DistortionModel::Type::NoDistortion)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void DistortionNull::distort(Eigen::Vector2d* point, Eigen::Matrix2d* out_jacobian) const
{
  if(out_jacobian)
    out_jacobian->setIdentity();
  return;
}

//--------------------------------------------------------------------------------------------------

void DistortionNull::getDistortionCoeffs(cv::Mat* coeffs_ptr) const
{
  assert(coeffs_ptr != NULL);
  *coeffs_ptr = cv::Mat(4,1,CV_64FC1, cv::Scalar::all(0));
}

//--------------------------------------------------------------------------------------------------

} // namespace camera
