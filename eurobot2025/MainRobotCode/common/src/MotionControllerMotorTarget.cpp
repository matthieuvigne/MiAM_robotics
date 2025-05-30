#include <common/MotionController.h>

bool MotionController::computeMotorTarget(Trajectory *traj,
                                          double const &timeInTrajectory,
                                          double const &dt,
                                          double const &slowDownRatio,
                                          DrivetrainMeasurements const &measurements,
                                          DrivetrainTarget &target)
{
    // Get current trajectory state.
    TrajectoryPoint targetPoint = traj->getCurrentPoint(curvilinearAbscissa_);

    // Update trajectory velocity based on lidar coeff.
    targetPoint.linearVelocity *= slowDownRatio;
    targetPoint.angularVelocity *= slowDownRatio;


    if (measurements.matchTime > 0.0)
    {
        log("MotionController.targetPositionX",targetPoint.position.x);
        log("MotionController.targetPositionY",targetPoint.position.y);
        log("MotionController.targetPositionTheta",targetPoint.position.theta);
        log("MotionController.targetVelocityLinear",targetPoint.linearVelocity);
        log("MotionController.targetVelocityAngular",targetPoint.angularVelocity);
    }

    // Compute targets for rotation and translation motors.
    BaseSpeed targetSpeed;

    // Feedforward.
    targetSpeed.linear = targetPoint.linearVelocity;
    targetSpeed.angular = targetPoint.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_.get();
    RobotPosition error = currentPosition - targetPoint.position;

    WheelSpeed speed = measurements.encoderPositionIncrement;
    speed.left /= dt;
    speed.right /= dt;
    BaseSpeed currentSpeed = kinematics_.forwardKinematics(speed, true);

    if (measurements.matchTime > 0.0)
    {
        log("MotionController.currentVelocityLinear", currentSpeed.linear);
        log("MotionController.currentVelocityAngular", currentSpeed.angular);
        log("MotionController.encoderVelocityRight", speed.right);
        log("MotionController.encoderVelocityLeft", speed.left);
    }

    // Rotate by -theta to express the error in the tangent frame.
    RobotPosition rotatedError = error.rotate(-targetPoint.position.theta);

    double trackingLongitudinalError = rotatedError.x;
    double trackingTransverseError = rotatedError.y;

    // Change sign if going backward.
    if (targetPoint.linearVelocity < 0)
        trackingTransverseError = -trackingTransverseError;

    double trackingAngleError = miam::trajectory::moduloTwoPi(currentPosition.theta - targetPoint.position.theta);


    // If we are beyond the end of the trajectory vector, look to see if we are close enough to the target point to stop.
    if (traj->getDuration() <= curvilinearAbscissa_ && currentTrajectories_.size() == 1)
    {
        if (trackingLongitudinalError < 2 && trackingAngleError < 0.01 && speed.right < 0.5 && speed.left < 0.5)
        {
            // Just stop the robot.
            target.motorSpeed.right = 0.0;
            target.motorSpeed.left = 0.0;
            return true;
        }
    }

    // Compute correction terms.

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    double pidLinearCorrection = 0.0;
    if (!(std::abs(targetPoint.angularVelocity) > 0.1 && std::abs(targetPoint.linearVelocity) < 0.1))
    {
        double const velocityError = currentSpeed.linear * std::cos(currentPosition.theta - targetPoint.position.theta) - targetSpeed.linear;
        pidLinearCorrection = PIDLinear_.computeValue(trackingLongitudinalError, velocityError, dt);
    }

    targetSpeed.linear += pidLinearCorrection;

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError;
    double transverseCorrection = 0.0;
    if (std::abs(targetPoint.linearVelocity) > 0.1 * robotParams_.maxWheelSpeedTrajectory)
        transverseCorrection = robotParams_.transverseKp * targetPoint.linearVelocity / robotParams_.maxWheelSpeedTrajectory * trackingTransverseError;
    if (targetPoint.linearVelocity < 0)
        transverseCorrection = -transverseCorrection;
    angularPIDError += transverseCorrection;

    targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, currentSpeed.angular - targetSpeed.angular, dt);

    // Invert velocity if playing on right side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // Convert from base velocity to motor wheel velocity.
    target.motorSpeed = kinematics_.inverseKinematics(targetSpeed);

    // Clamp to maximum speed
    double const maxAngularVelocity = 1.5 * robotParams_.maxWheelSpeedTrajectory / robotParams_.rightWheelRadius;
    if (target.motorSpeed.right > maxAngularVelocity)
        target.motorSpeed.right = maxAngularVelocity;
    if (target.motorSpeed.right < -maxAngularVelocity)
        target.motorSpeed.right = -maxAngularVelocity;
    if (target.motorSpeed.left > maxAngularVelocity)
        target.motorSpeed.left = maxAngularVelocity;
    if (target.motorSpeed.left < -maxAngularVelocity)
        target.motorSpeed.left = -maxAngularVelocity;


    if (measurements.matchTime > 0.0)
    {
        log("MotionController.trackingLongitudinalError",trackingLongitudinalError);
        log("MotionController.trackingTransverseError",trackingTransverseError);
        log("MotionController.trackingAngleError",trackingAngleError);
        log("MotionController.linearPIDCorrection",pidLinearCorrection);
        log("MotionController.angularPIDCorrection",PIDAngular_.getCorrection());
    }
    return false;
}