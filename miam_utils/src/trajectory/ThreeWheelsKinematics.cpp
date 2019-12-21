#include "miam_utils/trajectory/ThreeWheelsKinematics.hpp"
#include <cmath>

//======================================================================
// Constructors
//======================================================================

ThreeWheelsKinematics::ThreeWheelsKinematics(
  double const& length)
: length_(length)
{}

//======================================================================
// Public functions
//======================================================================

ThreeWheelsKinematics::BaseSpeed ThreeWheelsKinematics::forwardKinematics(
  WheelSpeed const& wheel_speed) const
{
  BaseSpeed base_speed;

  base_speed.angular_velocity_ =  (wheel_speed.wheel1_ +
    wheel_speed.wheel2_ + wheel_speed.wheel3_) / this->length_;

  base_speed.linear_velocity_y_ = - (wheel_speed.wheel1_ +
    base_speed.angular_velocity_ * this->length_);

  base_speed.linear_velocity_x_ = (wheel_speed.wheel3_ - 
    wheel_speed.wheel2_) / std::sqrt(3);

  return base_speed;
}

//----------------------------------------------------------------------

ThreeWheelsKinematics::WheelSpeed ThreeWheelsKinematics::inverseKinematics(
  BaseSpeed const& base_speed) const
{
  WheelSpeed wheel_speed;

  wheel_speed.wheel1_ = - base_speed.linear_velocity_y_ -
    base_speed.angular_velocity_ * this->length_;

  wheel_speed.wheel2_ = 0.5 * (base_speed.linear_velocity_y_ -
    std::sqrt(3) * base_speed.linear_velocity_x_) -
      base_speed.angular_velocity_ * this->length_;

  wheel_speed.wheel3_ = 0.5 * (base_speed.linear_velocity_y_ +
    std::sqrt(3) * base_speed.linear_velocity_x_) -
      base_speed.angular_velocity_ * this->length_;

  return wheel_speed;
}

//======================================================================
