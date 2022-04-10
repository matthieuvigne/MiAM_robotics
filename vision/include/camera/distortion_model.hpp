#ifndef VISION_DISTORTION_MODEL_HPP
#define VISION_DISTORTION_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <common/macros.hpp>

namespace camera {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class DistortionModel {
  
public:

  POINTER_TYPEDEF(DistortionModel);

  enum class Type {
    NoDistortion = 0,
    RadTan = 1,
    Fisheye = 2
  };

protected:

  DistortionModel() = delete;
  
  DistortionModel(
    Eigen::VectorXd const& distortion_coeffs,
    Type distortion_type);

  DistortionModel(DistortionModel const& model) = default;
  void operator=(DistortionModel const& model) = delete;

public:

  virtual ~DistortionModel(){}
  virtual std::string print() const;

public:

  // Common
  virtual bool operator==(DistortionModel const& model) const;
  inline Type getType() const;
  virtual void getDistortionCoeffs(cv::Mat* coeffs) const = 0;
  inline virtual int getNumCoeffs() const = 0;
  
  // Distort points
  virtual void distort(Eigen::Vector2d* point, Eigen::Matrix2d* jacobian) const = 0;
  void distort(Eigen::Vector2d const& point, Eigen::Vector2d* out_point, Eigen::Matrix2d* jacobian) const;
  
  // Undistort points
  virtual bool undistort(Eigen::Vector2d* point) const;
  bool undistort(Eigen::Vector2d const& point, Eigen::Vector2d* out_point) const;

protected:  

  Type const distortion_type_;
  Eigen::VectorXd const distortion_coeffs_;

}; // class DistortionModel  

//--------------------------------------------------------------------------------------------------
// Inline function definition
//--------------------------------------------------------------------------------------------------

DistortionModel::Type DistortionModel::getType() const
{
  return this->distortion_type_;
}

//--------------------------------------------------------------------------------------------------

} // namespace camera

#endif // VISION_DISTORTION_MODEL_HPP
