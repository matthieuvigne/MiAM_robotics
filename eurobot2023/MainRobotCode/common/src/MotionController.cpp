#include "common/MotionController.h"
#include <miam_utils/trajectory/Utilities.h>

MotionController::MotionController(RobotParameters const &robotParameters) : currentPosition_(),
                                                                             newTrajectories_(),
                                                                             currentTrajectories_(),
                                                                             wasTrajectoryFollowingSuccessful_(true),
                                                                             curvilinearAbscissa_(0.0),
                                                                             robotParams_(robotParameters)
{
    kinematics_ = DrivetrainKinematics(robotParams_.wheelRadius,
                                       robotParams_.wheelSpacing,
                                       robotParams_.encoderWheelRadius,
                                       robotParams_.encoderWheelSpacing);

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

void MotionController::init(RobotPosition const& startPosition, std::string const& teleplotPrefix)
{
    currentPosition_.set(startPosition);

    // Create logger.
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
    std::string filename = "logs/log" + std::string(timestamp) + "_" + robotParams_.name + ".hdf5";
    logger_.start(filename, teleplotPrefix);
    currentTime_ = 0.0;
}

miam::RobotPosition MotionController::getCurrentPosition()
{
    return currentPosition_.get();
}

void MotionController::resetPosition(miam::RobotPosition const &resetPosition, bool const &resetX, bool const &resetY, bool const &resetTheta)
{
    miam::RobotPosition position = currentPosition_.get();
    if (resetX)
        position.x = resetPosition.x;
    if (resetY)
        position.y = resetPosition.y;
    if (resetTheta)
        position.theta = resetPosition.theta;
    currentPosition_.set(position);
}

bool MotionController::setTrajectoryToFollow(std::vector<std::shared_ptr<Trajectory>> const &trajectories)
{
    newTrajectoryMutex_.lock();
    newTrajectories_ = trajectories;
    wasTrajectoryFollowingSuccessful_ = true;
    newTrajectoryMutex_.unlock();
    return true;
}

bool MotionController::waitForTrajectoryFinished()
{
    while (!isTrajectoryFinished())
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

DrivetrainTarget MotionController::computeDrivetrainMotion(DrivetrainMeasurements const &measurements,
                                                           double const &dt,
                                                           bool const &hasMatchStarted)
{
    // Log input
    currentTime_ += dt;
    log("timeIncrement",dt);
    log("encoderRight",measurements.encoderPosition[side::RIGHT]);
    log("encoderLeft",measurements.encoderPosition[side::LEFT]);
    log("motorVelocityRight", measurements.motorSpeed[side::RIGHT]);
    log("motorVelocityLeft", measurements.motorSpeed[side::LEFT]);

    // Odometry
    RobotPosition currentPosition = currentPosition_.update(kinematics_, measurements.encoderSpeed);
    log("currentPositionX",currentPosition.x);
    log("currentPositionY",currentPosition.y);
    log("currentPositionTheta",currentPosition.theta);

    BaseSpeed baseSpeed = kinematics_.forwardKinematics(measurements.encoderSpeed, true);
    log("currentVelocityLinear",baseSpeed.linear);
    log("currentVelocityAngular",baseSpeed.angular);

    DrivetrainTarget target;

    double slowDownCoeff = computeObstacleAvoidanceSlowdown(measurements.lidarDetection, hasMatchStarted);
    log("detectionCoeff",slowDownCoeff);

    // Load new trajectory, if needed.
    newTrajectoryMutex_.lock();
    if (!newTrajectories_.empty())
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

        if (curvilinearAbscissa_ > traj->getDuration())
        {
            // If we still have a trajectory after that, immediately switch to the next trajectory.
            if (currentTrajectories_.size() > 1)
            {
                // Not optimal, could be improved.
                currentTrajectories_.erase(currentTrajectories_.begin());
                traj = currentTrajectories_.at(0).get();
                curvilinearAbscissa_ = 0.0;
                std::cout << "Trajectory done, going to next one" << std::endl;
            }
        }

        // If we are more than a specified time  after the end of the trajectory, stop it anyway.
        // We hope to have servoed the robot is less than that anyway.
        if (curvilinearAbscissa_ - 0.1 > traj->getDuration())
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
            if (trajectoryDone && currentTrajectories_.size() == 1)
            {
                currentTrajectories_.erase(currentTrajectories_.begin());
                target.motorSpeed[0] = 0.0;
                target.motorSpeed[1] = 0.0;
                curvilinearAbscissa_ = 0.;

                // Reset avoidance counter
                avoidanceCount_ = 0;
            }
        }
    }

    // Force robot to stop if too close
    if (slowDownCoeff == 0)
        return target;

    if (!hasMatchStarted)
    {
        target.motorSpeed[side::RIGHT] = 0.0;
        target.motorSpeed[side::LEFT] = 0.0;
        PIDLinear_.resetIntegral(0.0);
        PIDAngular_.resetIntegral(0.0);
    }

    log("commandVelocityRight",target.motorSpeed[side::RIGHT]);
    log("commandVelocityLeft",target.motorSpeed[side::LEFT]);

    return target;
}

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

    log("targetPositionX",targetPoint.position.x);
    log("targetPositionY",targetPoint.position.y);
    log("targetPositionTheta",targetPoint.position.theta);
    log("targetLinearVelocity",targetPoint.linearVelocity);
    log("targetAngularVelocity",targetPoint.angularVelocity);

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

    double trackingLongitudinalError = rotatedError.x;
    double trackingTransverseError = rotatedError.y;

    // Change sign if going backward.
    if (targetPoint.linearVelocity < 0)
        trackingTransverseError = -trackingTransverseError;
    double trackingAngleError = miam::trajectory::moduloTwoPi(currentPosition.theta - targetPoint.position.theta);

    log("trackingLongitudinalError",trackingLongitudinalError);
    log("trackingTransverseError",trackingTransverseError);
    log("trackingAngleError",trackingAngleError);

    // If we are beyon trajector end, look to see if we are close enough to the target point to stop.
    if (traj->getDuration() <= curvilinearAbscissa_)
    {
        if (trackingLongitudinalError < 3 && trackingAngleError < 0.02 && measurements.motorSpeed[side::RIGHT] < 1.0 && measurements.motorSpeed[side::LEFT] < 1.0)
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
    if (std::abs(targetPoint.linearVelocity) > 0.1)
        targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError;
    double transverseCorrection = 0.0;
    if (std::abs(targetPoint.linearVelocity) > 0.1 * robotParams_.maxWheelSpeed)
        transverseCorrection = motioncontroller::transverseKp * targetPoint.linearVelocity / robotParams_.maxWheelSpeed * trackingTransverseError;
    if (targetPoint.linearVelocity < 0)
        transverseCorrection = -transverseCorrection;
    angularPIDError += transverseCorrection;

    // Use PID if a trajectory velocity is present.
    if (std::abs(targetPoint.linearVelocity) > 0.1 || std::abs(targetPoint.angularVelocity) > 0.005)
        targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

    // Invert velocity if playing on side::RIGHT side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // Convert from base velocity to motor wheel velocity.
    WheelSpeed wheelSpeed = kinematics_.inverseKinematics(targetSpeed);
    // Convert to motor unit.
    target.motorSpeed[side::RIGHT] = wheelSpeed.right;
    target.motorSpeed[side::LEFT] = wheelSpeed.left;

    // Clamp to maximum speed
    double const maxAngularVelocity = robotParams_.maxWheelSpeed / robotParams_.wheelRadius;
    for (int i = 0; i < 2; i++)
    {
        if (target.motorSpeed[i] > maxAngularVelocity)
            target.motorSpeed[i] = maxAngularVelocity;
        if (target.motorSpeed[i] < -maxAngularVelocity)
            target.motorSpeed[i] = -maxAngularVelocity;
    }


    log("linearPIDCorrection",PIDLinear_.getCorrection());
    log("angularPIDCorrection",PIDAngular_.getCorrection());

    return false;
}

double MotionController::computeObstacleAvoidanceSlowdown(std::deque<DetectedRobot> const &detectedRobots, bool const &hasMatchStarted)
{

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
    for (const DetectedRobot &robot : detectedRobots)
    {
        // Get the Lidar Point, symeterize it if needed and check its projection
        LidarPoint const point = this->isPlayingRightSide_
                                     ? LidarPoint(robot.point.r, -robot.point.theta)
                                     : LidarPoint(robot.point.r, robot.point.theta);

        if (!this->isLidarPointWithinTable(point))
            continue;
        nPointsInTable += 1;

        double x = point.r * std::cos(point.theta + (forward ? 0 : M_PI));
        double y = point.r * std::sin(point.theta + (forward ? 0 : M_PI));

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
                    const double current_coeff = std::min(1.0, std::max(0.2, (point.r - detection::r1) / (detection::r2 - detection::r1)));
                    if (current_coeff < coeff)
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
    if (!hasMatchStarted || currentTrajectories_.size() == 0)
    {
        numStopIters_ = 0;
        return coeff;
    }

    if (numStopIters_ >= minStopIters_ && numStopIters_ < minRestartIters_)
    {
        numStopIters_++;
        // Not ready to restart, just stop
        return 0.0;
    }

    if (is_robot_stopped)
    {
        numStopIters_++;
        if (numStopIters_ < minStopIters_)
        {
            // Not enough time spend, just slow down.
            coeff = 0.2;
        }
        else if (numStopIters_ > maxStopIters_)
        {
            avoidanceCount_++;
            std::cout << "avoidanceCount_ " << avoidanceCount_ << std::endl;

            if (avoidanceCount_ > maxAvoidanceAttempts_ || !currentTrajectories_.front()->isAvoidanceEnabled())
            {
                if (!currentTrajectories_.front()->isAvoidanceEnabled())
                {
                    std::cout << ">> MotionControllerAvoidance : Avoidance is disabled" << std::endl;
                }
                // Failed to perform avoidance.
                // Raise flag and end trajectory following.
                wasTrajectoryFollowingSuccessful_ = false;
                currentTrajectories_.clear();
                std::cout << "Obstacle still present, canceling trajectory" << std::endl;
                avoidanceCount_ = 0;
            }
            else
            {
                // Proceed avoidance
                // Search for the closest point along trajectory out of red zone
                // and far enough from the other robots and ask the MPC to set a new
                // trajectory to this point avoiding the fixed robot.

                std::vector<std::shared_ptr<Trajectory>> traj;
                RobotPosition targetPosition;
                RobotPosition endPosition;
                std::vector<RobotPosition> positions;
                // Set the new trajectory
                std::cout << ">> MotionControllerAvoidance : Triggering avoidance" << std::endl;
                numStopIters_ = 0;

                // Compute points to the left of the robot
                RobotPosition left_point(getCurrentPosition());
                // 40 cm to the left
                left_point.x -= 400 * sin(left_point.theta);
                left_point.y += 400 * cos(left_point.theta);
                // further 40 cm
                RobotPosition left_point_further(left_point);
                left_point_further.x += 400 * cos(left_point.theta);
                left_point_further.y += 400 * sin(left_point.theta);

                // Compute points to the right of the robot
                RobotPosition right_point(getCurrentPosition());
                // 40 cm to the right
                right_point.x += 400 * sin(right_point.theta);
                right_point.y -= 400 * cos(right_point.theta);
                // further 40 cm
                RobotPosition right_point_further(right_point);
                right_point_further.x += 400 * cos(right_point.theta);
                right_point_further.y += 400 * sin(right_point.theta);

                // Attempt left
                if (left_point.x < table_dimensions::table_max_x and left_point.x > table_dimensions::table_min_x and left_point.y < table_dimensions::table_max_y and left_point.y > table_dimensions::table_min_y)
                {

                    std::cout << ">> MotionControllerAvoidance : trying to go left" << std::endl;
                    std::cout << ">> MotionControllerAvoidance : current position : " << getCurrentPosition() << std::endl;
                    std::cout << ">> MotionControllerAvoidance : waypoint : " << left_point << std::endl;

                    targetPosition = getCurrentPosition();
                    positions.clear();
                    positions.push_back(targetPosition);
                    positions.push_back(left_point);
                    positions.push_back(left_point_further);
                    positions.push_back(currentTrajectories_.back()->getEndPoint().position);
                    traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3);

                    currentTrajectories_.clear();
                    currentTrajectories_ = traj;
                }
                else
                {
                    // Attempt right
                    if (right_point.x < table_dimensions::table_max_x and right_point.x > table_dimensions::table_min_x and right_point.y < table_dimensions::table_max_y and right_point.y > table_dimensions::table_min_y)
                    {

                        std::cout << ">> MotionControllerAvoidance : trying to go right" << std::endl;
                        std::cout << ">> MotionControllerAvoidance : current position : " << getCurrentPosition() << std::endl;
                        std::cout << ">> MotionControllerAvoidance : waypoint : " << right_point << std::endl;

                        targetPosition = getCurrentPosition();
                        positions.clear();
                        positions.push_back(targetPosition);
                        positions.push_back(right_point);
                        positions.push_back(right_point_further);
                        positions.push_back(currentTrajectories_.back()->getEndPoint().position);
                        traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3);

                        currentTrajectories_.clear();
                        currentTrajectories_ = traj;
                    }
                }
            }
        }
    }
    else
    {
        if (numStopIters_ >= minRestartIters_)
        {
            // Robot was stopped and is ready to start again.
            // Replan and retry trajectory.
            if (!currentTrajectories_.empty())
            {
                currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
                curvilinearAbscissa_ = 0;
                std::cout << "Continue trajectory; replan" << std::endl;
            }
        }
        numStopIters_ = 0;
    }

    //   std::cout << "numStopIters_ " << numStopIters_ << std::endl;
    //   std::cout << "avoidanceCount_ " << avoidanceCount_ << std::endl;

    return coeff;
}

bool MotionController::isLidarPointWithinTable(LidarPoint const &point)
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
    if (T_x_fi < table_dimensions::table_max_x and T_x_fi > table_dimensions::table_min_x and T_y_fi < table_dimensions::table_max_y and T_y_fi > table_dimensions::table_min_y)
    {
        return true;
    }

    return false;
}