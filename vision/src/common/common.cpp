#include <common/common.hpp>
#include <common/maths.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d getTWM()
{
  double half_marker_length = 0.10/2.0;
  Eigen::Affine3d TWM =
      Eigen::Translation3d(1.5, 1.0, 0.0)
    * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
  return TWM;
}

//--------------------------------------------------------------------------------------------------

Eigen::Vector3d getWpRi(Team team)
{
  // Parameters
  double const board_width = 3.0;
  double const board_height = 2.0;
  double const pole_height = 1.0;
  double const platform_width = 0.065;

  // Initialize the position
  Eigen::Vector3d WpR;
  switch(team)
  {
    case common::Team::UNKNOWN:
      WpR = Eigen::Vector3d{board_width/2., board_height, pole_height};
      break;
    case common::Team::PURPLE:
      WpR = Eigen::Vector3d{board_width/2. + platform_width, board_height, pole_height};
      break;
    case common::Team::YELLOW:
      WpR = Eigen::Vector3d{board_width/2. - platform_width, board_height, pole_height};
      break;
    default:
      throw std::runtime_error("Unknown team");
  }
  return WpR;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d getTRC(double azimuth_deg, double elevation_deg,
  Eigen::Matrix<double,6,1>* J_TRC_wrt_azimuth_ptr,
  Eigen::Matrix<double,6,1>* J_TRC_wrt_elevation_ptr)
{
  /* Sign conventions:
   * 500  = -90°  (looks left)
   * 1500 =   0°  (looks front)
   * 2500 = +90°  (looks right)
   */

  // Build the poses
  Eigen::Affine3d T1 =
        Eigen::AngleAxisd(azimuth_deg*RAD, Eigen::Vector3d::UnitY())
      * Eigen::Translation3d(Eigen::Vector3d{0.000,0.000,0.007});
  Eigen::Affine3d T2 =
        Eigen::AngleAxisd(-elevation_deg*RAD, Eigen::Vector3d::UnitX())
      * Eigen::Translation3d(Eigen::Vector3d{0.010,0.004,0.000})
      * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d TRC = T1 * T2;

  // Compute the Jacobians
  if(J_TRC_wrt_azimuth_ptr)
  {
    Eigen::Matrix<double,6,1>& J_TRC_wrt_azimuth = *J_TRC_wrt_azimuth_ptr;
    J_TRC_wrt_azimuth = Eigen::Matrix<double,6,1>::Unit(1)/DEG;
  }
  if(J_TRC_wrt_elevation_ptr)
  {
    Eigen::Matrix<double,6,1>& J_TRC_wrt_elevation = *J_TRC_wrt_elevation_ptr;
    Eigen::Matrix<double,6,6> J_TRC_wrt_T2 = common::so3r3::rightSe3ProductJacobian(T1, T2);
    Eigen::Matrix<double,6,1> J_T2_wrt_elevation = -Eigen::Matrix<double,6,1>::Unit(0)/DEG;
    J_TRC_wrt_elevation = J_TRC_wrt_T2 * J_T2_wrt_elevation;
  }

  return TRC;
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
