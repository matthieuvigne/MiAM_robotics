#include <common/common.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d getTWM()
{
  double half_marker_length = 0.10/2.0;
  Eigen::Affine3d TWM = 
      Eigen::Translation3d(1.5-half_marker_length,1.0+half_marker_length,0.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return TWM;
}

//--------------------------------------------------------------------------------------------------


} // namespace common
