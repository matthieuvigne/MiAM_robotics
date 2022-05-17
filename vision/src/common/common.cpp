#include <common/common.hpp>
#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d getTWM()
{
  double half_marker_length = 0.10/2.0;
  Eigen::Affine3d TWM = 
      Eigen::Translation3d(1.5 - half_marker_length, 1.0 + half_marker_length, 0.0)
    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return TWM;
}

//--------------------------------------------------------------------------------------------------

Eigen::Vector3d getWpCi(Team team)
{
  // Parameters
  double const board_width = 3.0;
  double const board_height = 2.0;
  double const pole_height = 1.0;
  double const platform_width = 0.20;

  // Initialize the position
  Eigen::Vector3d WpC;
  switch(team)
  {
    case common::Team::UNKNOWN:
      WpC = Eigen::Vector3d{board_width/2., board_height, pole_height};
      break;
    case common::Team::PURPLE:
      WpC = Eigen::Vector3d{board_width/2. + platform_width, board_height, pole_height};
      break;
    case common::Team::YELLOW:
      WpC = Eigen::Vector3d{board_width/2. - platform_width, board_height, pole_height};
      break;
    default:
      throw std::runtime_error("Unknown team");
  }
  return WpC;
}

//--------------------------------------------------------------------------------------------------

Eigen::Quaterniond getqRC(double azimuth_deg, double elevation_deg)
{
  /* Sign conventions:
   * 500  = -90°  (looks left)
   * 1500 =   0°  (looks front)
   * 2500 = +90°  (looks right)
   */
  Eigen::Quaterniond qRC;
  qRC = Eigen::AngleAxisd( azimuth_deg*RAD, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(-elevation_deg*RAD, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
  return qRC;
}

//--------------------------------------------------------------------------------------------------

Eigen::Quaterniond getqWR()
{
  Eigen::Quaterniond qWR;
  qWR = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
  return qWR;
}

//--------------------------------------------------------------------------------------------------


} // namespace common
