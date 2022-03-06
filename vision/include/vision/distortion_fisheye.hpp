#ifndef VISION_DISTORTION_FISHEYE_HPP
#define VISION_DISTORTION_FISHEYE_HPP

#include <vision/distortion_model.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class DistortionFisheye : public DistortionModel {

public:

  DISALLOW_EVIL_CONSTRUCTORS(DistortionFisheye);
  DistortionFisheye(Eigen::VectorXd const& distortion_coeffs);

public:

  inline int getNumCoeffs() const;
  void getDistortionCoeffs(cv::Mat* coeffs) const;
  void distort(Eigen::Vector2d* point, Eigen::Matrix2d* jacobian) const;

}; // class DistortionFisheye

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

int DistortionFisheye::getNumCoeffs() const
{
  return 4;
}

//--------------------------------------------------------------------------------------------------


} // namespace vision

#endif // VISION_DISTORTION_FISHEYE
