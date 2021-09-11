/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "Robot.h"

// Control of the robot motion on the table.
// Trajectory list handling, trajectory following, obstacle avoidance.

bool Robot::followTrajectory(Trajectory *traj, double const& curvilinearAbscissa_, double const & dt)
{
    // Get current trajectory state.
    trajectoryPoint_ = traj->getCurrentPoint(curvilinearAbscissa_);

    // Update trajectory velocity based on lidar coeff.
    trajectoryPoint_.linearVelocity *= coeff_;
    trajectoryPoint_.angularVelocity *= coeff_;

    // Compute targets for rotation and translation motors.
    BaseSpeed targetSpeed;

    // Feedforward.
    targetSpeed.linear = trajectoryPoint_.linearVelocity;
    targetSpeed.angular = trajectoryPoint_.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_.get();
    RobotPosition error = currentPosition - trajectoryPoint_.position;

    // Rotate by -theta to express the error in the tangent frame.
    RobotPosition rotatedError = error.rotate(-trajectoryPoint_.position.theta);

    trackingLongitudinalError_ = rotatedError.x;
    trackingTransverseError_ = rotatedError.y;

    // Change sign if going backward.
    if(trajectoryPoint_.linearVelocity < 0)
        trackingTransverseError_ = - trackingTransverseError_;
    trackingAngleError_ = miam::trajectory::moduloTwoPi(currentPosition.theta - trajectoryPoint_.position.theta);

    // If we are beyon trajector end, look to see if we are close enough to the target point to stop.
    if(traj->getDuration() <= curvilinearAbscissa_)
    {
        if(trackingLongitudinalError_ < 3 && trackingAngleError_ < 0.02 && motorSpeed_[RIGHT] < 100 && motorSpeed_[LEFT] < 100)
        {
            // Just stop the robot.
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            return true;
        }
    }

    // Compute correction terms.

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1)
        targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError_, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError_;
    double transverseCorrection = 0.0;
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 * robotdimensions::maxWheelSpeed)
        transverseCorrection = controller::transverseKp * trajectoryPoint_.linearVelocity / robotdimensions::maxWheelSpeed * trackingTransverseError_;
    if (trajectoryPoint_.linearVelocity < 0)
        transverseCorrection = - transverseCorrection;
    angularPIDError += transverseCorrection;

    // Use PID if a trajectory velocity is present.
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 || std::abs(trajectoryPoint_.angularVelocity) > 0.005 )
        targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

    // Invert velocity if playing on right side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // Convert from base velocity to motor wheel velocity.
    WheelSpeed wheelSpeed = kinematics_.inverseKinematics(targetSpeed);
    // Convert to motor unit.
    motorSpeed_[RIGHT] = wheelSpeed.right / robotdimensions::stepSize;
    motorSpeed_[LEFT] = wheelSpeed.left / robotdimensions::stepSize;

    return false;
}


void Robot::updateTrajectoryFollowingTarget(double const& dt)
{
    // Load new trajectories, if needed.
    if(!newTrajectories_.empty())
    {
        // We have new trajectories, erase the current trajectories and follow the new one.
        currentTrajectories_ = newTrajectories_;
        curvilinearAbscissa_ = 0;
        newTrajectories_.clear();
        std::cout << "Received new trajectory" << std::endl;
    }

    // If we have no trajectory to follow, do nothing.
    if(currentTrajectories_.empty())
    {
        motorSpeed_[0] = 0.0;
        motorSpeed_[1] = 0.0;
        curvilinearAbscissa_ = 0.;
    }
    else
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        // Look if first trajectory is done.
        // No avoidance if point turn.
        if (traj->getCurrentPoint(curvilinearAbscissa_).linearVelocity == 0)
            coeff_ = 1.0;

        curvilinearAbscissa_ += coeff_ * dt;
        if(curvilinearAbscissa_ > traj->getDuration())
        {
            // If we still have a trajectory after that, immediately switch to the next trajectory.
            if(currentTrajectories_.size() > 1)
            {
                // Not obtimal, could be improved.
                currentTrajectories_.erase(currentTrajectories_.begin());
                traj = currentTrajectories_.at(0).get();
                trajectoryStartTime_ = currentTime_;
                curvilinearAbscissa_ = 0.0;
                std::cout << "Trajectory done, going to next one" << std::endl;
            }
        }
        // If we are more than 1 second after the end of the trajectory, stop it anyway.
        // We hope to have servoed the robot is less than that anyway.
        if(curvilinearAbscissa_ - 0.1 >  traj->getDuration())
        {
            std::cout << "Timeout on trajectory following" << std::endl;
            currentTrajectories_.erase(currentTrajectories_.begin());
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            curvilinearAbscissa_ = 0.;
        }
        else
        {
            // Servo robot on current trajectory.
            bool trajectoryDone = followTrajectory(traj, curvilinearAbscissa_, dt);
            // If we finished the last trajectory, we can just end it straight away.
            if(trajectoryDone && currentTrajectories_.size() == 1)
            {
                currentTrajectories_.erase(currentTrajectories_.begin());
                motorSpeed_[0] = 0.0;
                motorSpeed_[1] = 0.0;
                curvilinearAbscissa_ = 0.;
            }
        }
    }

    // Read and clear error
    stepperMotors_.getError();
    // Send target to motors.
    stepperMotors_.setSpeed(motorSpeed_);
}

