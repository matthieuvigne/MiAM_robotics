#include <iostream>
#include <assert.h>

#include <vision/distortion_model.hpp>

namespace vision {
  
//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

DistortionModel::DistortionModel(
  Eigen::VectorXd const& distortion_coeffs,
  Type distortion_type)
: distortion_coeffs_  (distortion_coeffs),
  distortion_type_    (distortion_type)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool DistortionModel::operator==(DistortionModel const& model) const
{
  if(typeid(*this) != typeid(model))
    return false;
  if(this->distortion_coeffs_ != model.distortion_coeffs_)
    return false;
  return true;
}

//--------------------------------------------------------------------------------------------------

void DistortionModel::distort(
  Eigen::Vector2d const& point,
  Eigen::Vector2d* out_point,
  Eigen::Matrix2d* jacobian) const
{
  assert(out_point != NULL);
  *out_point = point;
  this->distort(out_point, jacobian);
}

//--------------------------------------------------------------------------------------------------

bool DistortionModel::undistort(Eigen::Vector2d* point) const
{
  assert(point != NULL);
  Eigen::Vector2d& y = *point;
  Eigen::Vector2d y_bar = y;
  Eigen::Matrix2d J;
  Eigen::Vector2d y_tmp;

  int i;
  int constexpr max_iters = 30;
  double constexpr tolerance = 1e-8;
  for(i=0; i<max_iters; ++i)
  {
    y_tmp = y_bar;
    this->distort(&y_tmp, &J);
    Eigen::Vector2d e(y - y_tmp);
    Eigen::Vector2d du = (J.transpose()*J).inverse() * J.transpose() * e;
    y_bar += du;
    if(e.dot(e) <= tolerance)
      break;
  }

  y = y_bar;
  return (i<max_iters);
}

//--------------------------------------------------------------------------------------------------

bool DistortionModel::undistort(
  Eigen::Vector2d const& point,
  Eigen::Vector2d* out_point) const
{
  assert(out_point != NULL);
  *out_point = point;
  return this->undistort(out_point);
}

//--------------------------------------------------------------------------------------------------

std::string DistortionModel::print() const
{
  std::stringstream out;
  std::string distortion_type;
  std::stringstream distortion_coeffs;
  switch(this->distortion_type_)
  {
    case Type::NoDistortion:
    {
      distortion_type = "no distortion";
      distortion_coeffs << "{}";
      break;
    }
    case Type::RadTan:
    {
      distortion_type = "radial-tangential";
      double const& k1 = this->distortion_coeffs_[0];
      double const& k2 = this->distortion_coeffs_[1];
      double const& p1 = this->distortion_coeffs_[2];
      double const& p2 = this->distortion_coeffs_[3];
      double const& k3 = this->distortion_coeffs_[4];
      distortion_coeffs << "{ k1=" << k1 << ", k2=" << k2 << ", p1=" << p1 << ", p2=" << p2 
        << ", k3=" << k3 << " }";
      break;
    }
    case Type::Fisheye:
    {
      distortion_type = "fisheye";
      double const& k1 = this->distortion_coeffs_[0];
      double const& k2 = this->distortion_coeffs_[1];
      double const& k3 = this->distortion_coeffs_[2];
      double const& k4 = this->distortion_coeffs_[3];
      distortion_coeffs << "{ k1=" << k1 << ", k2=" << k2 << ", k3=" << k3 << ", k4=" << k4 << " }";
      break;
    }
  }
  out << "- Distortion type: " << distortion_type << std::endl;
  out << "- Distortion coeffs: " << distortion_coeffs.str();
  return out.str();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
