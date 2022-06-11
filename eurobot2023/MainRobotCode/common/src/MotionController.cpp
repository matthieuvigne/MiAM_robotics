#include "MotionController.h"
#include <miam_utils/trajectory/Utilities.h>


MotionController::MotionController():
    currentPosition_(),
    newTrajectories_(),
    currentTrajectories_(),
    wasTrajectoryFollowingSuccessful_(true),
    curvilinearAbscissa_(0.0)
{
    kinematics_ = DrivetrainKinematics(robotdimensions::wheelRadius,
                                       robotdimensions::wheelSpacing,
                                       robotdimensions::encoderWheelRadius,
                                       robotdimensions::encoderWheelSpacing);
    // Update trajectory config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 120;
    initialPosition.y = 1200;
    initialPosition.theta = 0;
    currentPosition_.set(initialPosition);

    // Set PIDs.
    PIDLinear_ = miam::PID(motioncontroller::linearKp, motioncontroller::linearKd, motioncontroller::linearKi, 0.2);
    PIDAngular_ = miam::PID(motioncontroller::rotationKp, motioncontroller::rotationKd, motioncontroller::rotationKi, 0.15);

}


miam::RobotPosition MotionController::getCurrentPosition()
{
    return currentPosition_.get();
}

void MotionController::resetPosition(miam::RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta)
{
    miam::RobotPosition position = currentPosition_.get();
    if(resetX)
        position.x = resetPosition.x;
    if(resetY)
        position.y = resetPosition.y;
    if(resetTheta)
        position.theta = resetPosition.theta;
    currentPosition_.set(position);
}


bool MotionController::setTrajectoryToFollow(std::vector<std::shared_ptr<Trajectory>> const& trajectories)
{
    newTrajectoryMutex_.lock();
    newTrajectories_ = trajectories;
    wasTrajectoryFollowingSuccessful_ = true;
    newTrajectoryMutex_.unlock();
    return true;
}


bool MotionController::waitForTrajectoryFinished()
{
    while(!isTrajectoryFinished())
        usleep(15000);
    return wasTrajectoryFollowingSuccessful_;
}


bool MotionController::isTrajectoryFinished()
{
    return (currentTrajectories_.size() == 0 && newTrajectories_.size() == 0);
}

bool MotionController::wasTrajectoryFollowingSuccessful()
{
    return wasTrajectoryFollowingSuccessful_;
}



DrivetrainTarget MotionController::computeDrivetrainMotion(DrivetrainMeasurements const& measurements, double const& dt)
{
    // Odometry
    currentPosition_.update(kinematics_, measurements.encoderSpeed);


    DrivetrainTarget target;

    double slowDownCoeff = computeObstacleAvoidanceSlowdown(measurements.lidarDetection);

    // FIXME
    // Force stop robot
    if (slowDownCoeff == 0)
        return target;

    // Load new trajectory, if needed.
    newTrajectoryMutex_.lock();
    if(!newTrajectories_.empty())
    {
        // We have new trajectories, erase the current trajectories and follow the new one.
        currentTrajectories_ = newTrajectories_;
        curvilinearAbscissa_ = 0;
        newTrajectories_.clear();
        std::cout << "Received new trajectory" << std::endl;
    }
    newTrajectoryMutex_.unlock();

    // If we have no trajectory to follow, do nothing.
    if (!currentTrajectories_.empty())
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        // No avoidance if point turn.
        if (traj->getCurrentPoint(curvilinearAbscissa_).linearVelocity == 0)
            slowDownCoeff = 1.0;
        curvilinearAbscissa_ += slowDownCoeff * dt;

        if(curvilinearAbscissa_ > traj->getDuration())
        {
            // If we still have a trajectory after that, immediately switch to the next trajectory.
            if(currentTrajectories_.size() > 1)
            {
                // Not obtimal, could be improved.
                currentTrajectories_.erase(currentTrajectories_.begin());
                traj = currentTrajectories_.at(0).get();
                curvilinearAbscissa_ = 0.0;
                std::cout << "Trajectory done, going to next one" << std::endl;
            }
        }

        // If we are more than a specified time  after the end of the trajectory, stop it anyway.
        // We hope to have servoed the robot is less than that anyway.
        if(curvilinearAbscissa_ - 0.1 >  traj->getDuration())
        {
            std::cout << "Timeout on trajectory following" << std::endl;
            currentTrajectories_.erase(currentTrajectories_.begin());
            curvilinearAbscissa_ = 0.;
        }
        else
        {
            // Servo robot on current trajectory.
            bool trajectoryDone = computeMotorTarget(traj, curvilinearAbscissa_, dt, slowDownCoeff, measurements, target);
            // If we finished the last trajectory, we can just end it straight away.
            if(trajectoryDone && currentTrajectories_.size() == 1)
            {
                currentTrajectories_.erase(currentTrajectories_.begin());
                target.motorSpeed[0] = 0.0;
                target.motorSpeed[1] = 0.0;
                curvilinearAbscissa_ = 0.;
            }
        }
    }
    return target;
}



bool MotionController::computeMotorTarget(Trajectory *traj,
                                          double const& timeInTrajectory,
                                          double const& dt,
                                          double const& slowDownRatio,
                                          DrivetrainMeasurements const& measurements,
                                          DrivetrainTarget &target)
{
    // Get current trajectory state.
    TrajectoryPoint targetPoint = traj->getCurrentPoint(curvilinearAbscissa_);

    // Update trajectory velocity based on lidar coeff.
    targetPoint.linearVelocity *= slowDownRatio;
    targetPoint.angularVelocity *= slowDownRatio;

    // Compute targets for rotation and translation motors.
    BaseSpeed targetSpeed;

    // Feedforward.
    targetSpeed.linear = targetPoint.linearVelocity;
    targetSpeed.angular = targetPoint.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_.get();
    RobotPosition error = currentPosition - targetPoint.position;

    // Rotate by -theta to express the error in the tangent frame.
    RobotPosition rotatedError = error.rotate(-targetPoint.position.theta);

    double trackingLongitudinalError_ = rotatedError.x;
    double trackingTransverseError_ = rotatedError.y;

    // Change sign if going backward.
    if(targetPoint.linearVelocity < 0)
        trackingTransverseError_ = - trackingTransverseError_;
    double trackingAngleError_ = miam::trajectory::moduloTwoPi(currentPosition.theta - targetPoint.position.theta);

    // If we are beyon trajector end, look to see if we are close enough to the target point to stop.
    if(traj->getDuration() <= curvilinearAbscissa_)
    {
        if(trackingLongitudinalError_ < 3 && trackingAngleError_ < 0.02 && measurements.motorSpeed[RIGHT] < 1.0 && measurements.motorSpeed[LEFT] < 1.0)
        {
            // Just stop the robot.
            target.motorSpeed[0] = 0.0;
            target.motorSpeed[1] = 0.0;
            return true;
        }
    }

    // Compute correction terms.

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    if(std::abs(targetPoint.linearVelocity) > 0.1)
        targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError_, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError_;
    double transverseCorrection = 0.0;
    if(std::abs(targetPoint.linearVelocity) > 0.1 * robotdimensions::maxWheelSpeed)
        transverseCorrection = motioncontroller::transverseKp * targetPoint.linearVelocity / robotdimensions::maxWheelSpeed * trackingTransverseError_;
    if (targetPoint.linearVelocity < 0)
        transverseCorrection = - transverseCorrection;
    angularPIDError += transverseCorrection;

    // Use PID if a trajectory velocity is present.
    if(std::abs(targetPoint.linearVelocity) > 0.1 || std::abs(targetPoint.angularVelocity) > 0.005 )
        targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

    // Invert velocity if playing on right side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // Convert from base velocity to motor wheel velocity.
    WheelSpeed wheelSpeed = kinematics_.inverseKinematics(targetSpeed);
    // Convert to motor unit.
    target.motorSpeed[RIGHT] = wheelSpeed.right / robotdimensions::stepSize;
    target.motorSpeed[LEFT] = wheelSpeed.left / robotdimensions::stepSize;

    return false;
}


double MotionController::computeObstacleAvoidanceSlowdown(std::deque<DetectedRobot> const& detectedRobots)
{
  // Handle robot stops
  static int num_stop_iters = 0.;
  constexpr int min_stop_iters = 12; // Minimum number of iterations to stop, i.e 10ms.
  constexpr int min_restart_iter = 20; // Minimum number of iterations to restart, i.e 10ms.

  double coeff = 1.0;
  bool is_robot_stopped = false;

  LidarPoint detected_point;
  detected_point.r = 1e6;
  detected_point.theta = 0.;

  // Update trajectory.
  bool forward = true;
  if (!currentTrajectories_.empty())
  {
    miam::trajectory::TrajectoryPoint trajectoryPoint = currentTrajectories_.at(0)->getCurrentPoint(curvilinearAbscissa_);
    forward = (trajectoryPoint.linearVelocity >= 0);
  }

  int nPointsInTable = 0;
  for(const DetectedRobot& robot : detectedRobots)
  {
    // Get the Lidar Point, symeterize it if needed and check its projection
    LidarPoint const point = this->isPlayingRightSide_
      ? LidarPoint(robot.point.r, -robot.point.theta)
      : LidarPoint(robot.point.r, robot.point.theta);

    if(!this->isLidarPointWithinTable(point)) continue;
    nPointsInTable += 1;

    double x = point.r * std::cos(point.theta + (forward ? 0: M_PI));
    double y = point.r * std::sin(point.theta + (forward ? 0: M_PI));

    if (x > 0)
    {
        if (x < detection::x_max)
        {
            if (std::abs(y) < detection::y_max)
            {
                // Stop robot.
                coeff = 0.0;
                is_robot_stopped = true;
                detected_point = point;
            }
        }
        else if (x < detection::xfar_max)
        {
            double maximumY = detection::y_max + (x - detection::x_max) / (detection::xfar_max - detection::x_max) * (detection::yfar_max - detection::y_max);
            if (std::abs(y) < maximumY)
            {
                const double current_coeff = std::min(1.0, std::max(0.2, (point.r - detection::r1)
                    / (detection::r2 - detection::r1)));
                if(current_coeff < coeff)
                {
                    coeff = current_coeff;
                    detected_point = point;
                }
            }
        }
    }
  }

  // Before match: just return coeff, don't trigger memory.

  // FIXME !
  bool hasMatchStarted_ = true;
  if (!hasMatchStarted_ || currentTrajectories_.size() == 0)
  {
    num_stop_iters = 0;
    return coeff;
  }

    if (num_stop_iters >= min_stop_iters &&  num_stop_iters < min_restart_iter)
    {
        num_stop_iters++;
        // Not ready to restart, just stop
        return 0.0;
    }

    // FIXME
    int const avoidanceTimeout_ = 50;
  if (is_robot_stopped)
  {
    num_stop_iters++;
    if (num_stop_iters < min_stop_iters)
    {
        // Not enough time spend, just slow down.
        coeff = 0.2;
    }
    else if(num_stop_iters > avoidanceTimeout_)
    {
      // Proceed avoidance
      // Search for the closest point along trajectory out of red zone
      // and far enough from the other robots and ask the MPC to set a new
      // trajectory to this point avoiding the fixed robot.
      // Set the new trajectory
      //~ std::cout << num_stop_iters << std::endl;
      num_stop_iters = 0;

      // Failed to perform avoidance.
      // Raise flag and end trajectory following.
      wasTrajectoryFollowingSuccessful_ = false;
      currentTrajectories_.clear();
      std::cout << "Obstacle still present, canceling trajectory" << std::endl;
    }
  }
  else
  {
      if (num_stop_iters >= min_restart_iter)
      {
          // Robot was stopped and is ready to start again.
          // Replan and retry trajectory.
          if (!currentTrajectories_.empty())
          {
              currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
              curvilinearAbscissa_ = 0;
          }
      }
      num_stop_iters = 0;
  }

  return coeff;
}


bool MotionController::isLidarPointWithinTable(LidarPoint const& point)
{
  // 1 Get the robot current position
  miam::RobotPosition const robot_position = this->getCurrentPosition();
  double const T_x_R = robot_position.x;
  double const T_y_R = robot_position.y;
  double const theta_T_R = robot_position.theta;

  // 2. Project the Lidar point within the table
  double const T_x_fi = T_x_R + point.r * std::cos(theta_T_R + point.theta);
  double const T_y_fi = T_y_R + point.r * std::sin(theta_T_R + point.theta);


  // 3. Check if the lidar point falls within the table
  if(T_x_fi < table_dimensions::table_max_x and T_x_fi > table_dimensions::table_min_x
    and T_y_fi < table_dimensions::table_max_y and T_y_fi > table_dimensions::table_min_y )
  {
    return true;
  }

  return false;
}