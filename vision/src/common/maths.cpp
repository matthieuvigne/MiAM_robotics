#include <eigen3/Eigen/Core>

#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

double convertDegreeToRadian(double angle_deg)
{
  return M_PI*angle_deg/180.;
}

//--------------------------------------------------------------------------------------------------

double convertRadianToDegree(double angle_rad)
{
  return 180.*angle_rad/M_PI;
}

//--------------------------------------------------------------------------------------------------

} // namespace common
