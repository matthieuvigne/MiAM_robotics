/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/DrivetrainKinematics.h"
#include <cmath>

DrivetrainKinematics::DrivetrainKinematics():
        rightMotorWheelRadius_(1.0),
        leftMotorWheelRadius_(1.0),
        motorWheelSpacing_(1.0),
        rightEncoderWheelRadius_(1.0),
        leftEncoderWheelRadius_(1.0),
        encoderWheelSpacing_(1.0)
{
}

DrivetrainKinematics::DrivetrainKinematics(double const& rightMotorWheelRadiusIn,
                                           double const& leftMotorWheelRadiusIn,
                                           double const& motorWheelSpacingIn,
                                           double const& rightEncoderWheelRadiusIn,
                                           double const& leftEncoderWheelRadiusIn,
                                           double const& encoderWheelSpacingIn):
        rightMotorWheelRadius_(rightMotorWheelRadiusIn),
        leftMotorWheelRadius_(leftMotorWheelRadiusIn),
        motorWheelSpacing_(motorWheelSpacingIn),
        rightEncoderWheelRadius_(rightEncoderWheelRadiusIn),
        leftEncoderWheelRadius_(leftEncoderWheelRadiusIn),
        encoderWheelSpacing_(encoderWheelSpacingIn)
{
}

BaseSpeed DrivetrainKinematics::forwardKinematics(WheelSpeed const& wheelSpeedIn, bool const& useEncoders) const
{
    BaseSpeed speed;
    double const rightWheelRadius = (useEncoders ? rightEncoderWheelRadius_ : rightMotorWheelRadius_);
    double const leftWheelRadius = (useEncoders ? leftEncoderWheelRadius_ : leftMotorWheelRadius_);
    double const wheelSpacing = (useEncoders ? encoderWheelSpacing_ : motorWheelSpacing_);

    double const rightVelocity = wheelSpeedIn.right * rightWheelRadius;
    double const leftVelocity = wheelSpeedIn.left * leftWheelRadius;
    // Linear velocity: average of both velocities.
    speed.linear = (rightVelocity + leftVelocity) / 2.0;
    // Angular velocity: difference of both velocities over wheel spacing.
    speed.angular = (rightVelocity - leftVelocity) / 2.0 / wheelSpacing;
    return speed;
}

WheelSpeed DrivetrainKinematics::inverseKinematics(BaseSpeed const& baseSpeedIn, bool const& useEncoders) const
{
    WheelSpeed speed;
    double const rightWheelRadius = (useEncoders ? rightEncoderWheelRadius_ : rightMotorWheelRadius_);
    double const leftWheelRadius = (useEncoders ? leftEncoderWheelRadius_ : leftMotorWheelRadius_);

    double wheelSpacing = (useEncoders ? encoderWheelSpacing_ : motorWheelSpacing_);

    speed.right = (baseSpeedIn.linear + wheelSpacing * baseSpeedIn.angular) / rightWheelRadius;
    speed.left = (baseSpeedIn.linear - wheelSpacing * baseSpeedIn.angular) / leftWheelRadius;
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
