/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/DrivetrainKinematics.h"
#include <cmath>

DrivetrainKinematics::DrivetrainKinematics():
        motorWheelRadius_(1.0),
        motorWheelSpacing_(1.0),
        encoderWheelRadius_(1.0),
        encoderWheelSpacing_(1.0)
{
}

DrivetrainKinematics::DrivetrainKinematics(double const& motorWheelRadiusIn,
                                           double const& motorWheelSpacingIn,
                                           double const& encoderWheelRadiusIn,
                                           double const& encoderWheelSpacingIn):
        motorWheelRadius_(motorWheelRadiusIn),
        motorWheelSpacing_(motorWheelSpacingIn),
        encoderWheelRadius_(encoderWheelRadiusIn),
        encoderWheelSpacing_(encoderWheelSpacingIn)
{
}

BaseSpeed DrivetrainKinematics::forwardKinematics(WheelSpeed const& wheelSpeedIn, bool const& useEncoders) const
{
    BaseSpeed speed;
    double wheelRadius = (useEncoders ? encoderWheelRadius_ : motorWheelRadius_);
    double wheelSpacing = (useEncoders ? encoderWheelSpacing_ : motorWheelSpacing_);

    // Linear velocity: average of both velocities.
    speed.linear = (wheelSpeedIn.right + wheelSpeedIn.left) / 2.0 * wheelRadius;
    // Angular velocity: difference of both velocities over wheel spacing.
    speed.angular = (wheelSpeedIn.right - wheelSpeedIn.left) / 2.0 * wheelRadius / wheelSpacing;
    return speed;
}

WheelSpeed DrivetrainKinematics::inverseKinematics(BaseSpeed const& baseSpeedIn, bool const& useEncoders) const
{
    WheelSpeed speed;
    double wheelRadius = (useEncoders ? encoderWheelRadius_ : motorWheelRadius_);
    double wheelSpacing = (useEncoders ? encoderWheelSpacing_ : motorWheelSpacing_);

    speed.right = (baseSpeedIn.linear + wheelSpacing * baseSpeedIn.angular) / wheelRadius;
    speed.left = (baseSpeedIn.linear - wheelSpacing * baseSpeedIn.angular) / wheelRadius;
    return speed;
}


void DrivetrainKinematics::integratePosition(WheelSpeed const& wheelSpeedIn, miam::RobotPosition & positionInOut, bool const& useEncoders) const
{
    // Compute base velocity.
    BaseSpeed speed = forwardKinematics(wheelSpeedIn, useEncoders);
    // Integrate into position.
    positionInOut.theta += speed.angular;
    positionInOut.x += std::cos(positionInOut.theta) * speed.linear;
    positionInOut.y += std::sin(positionInOut.theta) * speed.linear;
}
