#ifndef VISION_DISTORTION_NULL_HPP
#define VISION_DISTORTION_NULL_HPP

#include <vision/distortion_model.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class DistortionNull : public DistortionModel {

public:

  DistortionNull();

public:

  inline int getNumCoeffs() const;
  void getDistortionCoeffs(cv::Mat* coeffs) const;
  void distort(Eigen::Vector2d* point, Eigen::Matrix2d* jacobian) const;

}; // class DistortionNull

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

int DistortionNull::getNumCoeffs() const
{
  return 0;
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_DISTORTION_NULL_HPP
