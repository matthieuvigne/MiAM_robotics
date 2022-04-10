#ifndef VISION_DISTORTION_FISHEYE_HPP
#define VISION_DISTORTION_FISHEYE_HPP

#include <camera/distortion_model.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class DistortionFisheye : public DistortionModel {

public:

  DISALLOW_EVIL_CONSTRUCTORS(DistortionFisheye);
  DistortionFisheye(Eigen::VectorXd const& distortion_coeffs);
  enum {k1, k2, k3, k4};

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


} // namespace camera

#endif // VISION_DISTORTION_FISHEYE
