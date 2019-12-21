#ifndef MIAM_THREE_WHEELS_KINEMATICS_HPP
#define MIAM_THREE_WHEELS_KINEMATICS_HPP

#include "miam_utils/trajectory/RobotPosition.h"

class ThreeWheelsKinematics {

public:

  struct BaseSpeed
  {
    BaseSpeed()
    : angular_velocity_(0.),
      linear_velocity_x_(0.),
      linear_velocity_y_(0.)
    {}

    BaseSpeed(
      double const& angular_velocity,
      double const& linear_velocity_x,
      double const& linear_velocity_y)
    : angular_velocity_(angular_velocity),
      linear_velocity_x_(linear_velocity_x),
      linear_velocity_y_(linear_velocity_y)
    {}

    double angular_velocity_;
    double linear_velocity_x_;
    double linear_velocity_y_;
  };
  
  struct WheelSpeed
  {
    WheelSpeed()
    : wheel1_(0.),
      wheel2_(0.),
      wheel3_(0.)
    {}

    WheelSpeed(
      double const& wheel1,
      double const& wheel2,
      double const& wheel3)
    : wheel1_(wheel1),
      wheel2_(wheel2),
      wheel3_(wheel3)
    {}

    double wheel1_;
    double wheel2_;
    double wheel3_;
  };

public:

  ThreeWheelsKinematics() = delete;
  ThreeWheelsKinematics(double const& length);

public:

  /// \brief Forward kinematic model
  BaseSpeed forwardKinematics(WheelSpeed const& speed) const;

  /// \brief Inverse kinematic model
  WheelSpeed inverseKinematics(BaseSpeed const& speed) const;

private:

  double length_; ///< Lenght from center to wheel

}; // class ThreeWheelsKinematics

#endif // MIAM_THREE_WHEELS_KINEMATICS_HPP
