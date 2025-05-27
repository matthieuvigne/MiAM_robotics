#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include "common/ThreadHandler.h"

// Time to keep servoing after the theoretical end of the trajectory.
# define TRAJECTORY_TRACKING_TIMEOUT 0.5

MotionController::MotionController(RobotParameters const &robotParameters, Logger *logger) :
    robotParams_(robotParameters),
    motionPlanner_(logger),
    logger_(logger)
{
    kinematics_ = DrivetrainKinematics(robotParams_.rightWheelRadius,
                                       robotParams_.leftWheelRadius,
                                       robotParams_.wheelSpacing,
                                       robotParams_.rightEncoderWheelRadius,
                                       robotParams_.leftEncoderWheelRadius,
                                       robotParams_.encoderWheelSpacing);
}

void MotionController::init(RobotPosition const& startPosition)
{
    currentPosition_.set(startPosition);
    currentTime_ = 0.0;

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 120;
    initialPosition.y = 1200;
    initialPosition.theta = 0;
    currentPosition_.set(initialPosition);

    // Set PIDs.
    PIDLinear_ = miam::PID(robotParams_.linearKp, robotParams_.linearKd, robotParams_.linearKi, 0.2);
    PIDAngular_ = miam::PID(robotParams_.rotationKp, robotParams_.rotationKd, robotParams_.rotationKi, 0.15);

    // Set MotionPlanner

    // Default avoidance mode
    // avoidanceMode_ = AvoidanceMode::AVOIDANCE_BASIC;
    avoidanceMode_ = AvoidanceMode::AVOIDANCE_OFF;
    avoidanceCount_ = 0;
    isStopped_ = false;

    // Create solver thread
    avoidanceComputationScheduled_ = false;

    std::thread stratSolver(&MotionController::loopOnAvoidanceComputation, this);
    ThreadHandler::addThread(stratSolver);

    // Init controller state
    motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;
    timeSinceFirstStopped_ = std::chrono::steady_clock::now();
    timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
}

miam::RobotPosition MotionController::getCurrentPosition() const
{
    return currentPosition_.get();
}

miam::trajectory::TrajectoryVector MotionController::getCurrentTrajectories()
{
    return currentTrajectories_;
}

double MotionController::getCurvilinearAbscissa()
{
    return curvilinearAbscissa_;
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

bool MotionController::setTrajectoryToFollow(TrajectoryVector const &trajectories)
{
    newTrajectoryMutex_.lock();
    newTrajectories_ = trajectories;
    wasTrajectoryFollowingSuccessful_ = true;
    if (!newTrajectories_.empty())
        currentTargetEndPosition_ = newTrajectories_.back()->getEndPoint().position;
    newTrajectoryMutex_.unlock();
    return true;
}

void MotionController::stopCurrentTrajectoryTracking()
{
    newTrajectoryMutex_.lock();
    askedForTrackingStop_ = true;
    newTrajectoryMutex_.unlock();
}

bool MotionController::waitForTrajectoryFinished()
{
    while (!isTrajectoryFinished())
        usleep(2000);
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

void MotionController::startMatch(RobotPosition const& startPosition)
{
    resetPosition(startPosition);
    PIDLinear_.resetIntegral(0.0);
    PIDAngular_.resetIntegral(0.0);
}


DrivetrainTarget MotionController::computeDrivetrainMotion(DrivetrainMeasurements const &measurements,
                                                           double const &dt)
{
    // Log input
    currentTime_ += dt;
    log("timeIncrement",dt);


    // Odometry
    RobotPosition currentPosition = currentPosition_.update(kinematics_, measurements.encoderPositionIncrement);


    // Only log after match start
    if (measurements.matchTime > 0.0)
    {
        log("MotionController.encoderRight",measurements.encoderPosition.right);
        log("MotionController.encoderLeft",measurements.encoderPosition.left);
        log("MotionController.motorVelocityRight", measurements.motorSpeed.right);
        log("MotionController.motorVelocityLeft", measurements.motorSpeed.left);
        log("MotionController.currentPositionX",currentPosition.x);
        log("MotionController.currentPositionY",currentPosition.y);
        log("MotionController.currentPositionTheta",currentPosition.theta);
    }

    DrivetrainTarget target;

    // Update list of obstacles
    detectedObstaclesMutex_.lock();
    detectedObstacles_.clear();
    displayDetectedObstacles_.clear();

    // add obstacles from measurements
    int nObstaclesOnTable = 0;
    for (auto robot : measurements.lidarDetection)
    {
        // Get the Lidar Point, symeterize it if needed and check its projection
        LidarPoint const point = this->isPlayingRightSide_
                                    ? LidarPoint(robot.point.r, -robot.point.theta)
                                    : LidarPoint(robot.point.r, robot.point.theta);

        RobotPosition obspos = lidarPointToRobotPosition(point);

        if (!this->isLidarPointWithinTable(point))
        {
            obspos.theta = M_PI;
            displayDetectedObstacles_.push_back(obspos);
            continue;
        }
        displayDetectedObstacles_.push_back(obspos);

        // Ignore obstacles for the first second of the match
        if (measurements.matchTime > 1.0 && isDetectionEnabled_)
        {
            detectedObstacles_.push_back(std::make_tuple(obspos, detection::mpc_obstacle_size));
            // Log
            if (nObstaclesOnTable < 5)
            {
                log("ObstacleX_" + std::to_string(nObstaclesOnTable), obspos.x);
                log("ObstacleY_" + std::to_string(nObstaclesOnTable), obspos.y);
                log("ObstacleN_" + std::to_string(nObstaclesOnTable), robot.nPoints);
            }
            nObstaclesOnTable += 1;
        }
    }
    // Log obstacles
    log("lidarNumberOfObstacles", nObstaclesOnTable);

    // Update game state based on other robot
    if (measurements.matchTime > 1.0)
        gameState_.detectOtherRobotAction(detectedObstacles_, measurements.matchTime, logger_);

    // add obstacles
    for (auto obstacle : getPersistentObstacles())
    {
        detectedObstacles_.push_back(obstacle);
    }


    detectedObstaclesMutex_.unlock();

    // Compute slowdown
    // slowDownCoeff_ = computeObstacleAvoidanceSlowdown(measurements.lidarDetection);
    slowDownCoeff_ = computeObstacleAvoidanceSlowdownAnticipateTrajectory();
    clampedSlowDownCoeff_ = std::min(slowDownCoeff_, clampedSlowDownCoeff_ + 0.05);

    changeMotionControllerState();

    target = resolveMotionControllerState(measurements, dt, measurements.matchTime > 0);


    if (measurements.matchTime > 0.0)
    {
        log("motionControllerState", static_cast<int>(motionControllerState_));
        log("detectionCoeff",slowDownCoeff_);
        log("clampedDetectionCoeff", clampedSlowDownCoeff_);
        log("MotionController.rawCommandVelocityRight",target.motorSpeed.right);
        log("MotionController.rawCommandVelocityLeft",target.motorSpeed.left);
    }

    // Clamp target acceleration
    double const maxAccel = 2.0 * robotParams_.maxWheelAccelerationTrajectory / robotParams_.rightWheelRadius;
    target.motorSpeed.right = std::clamp(target.motorSpeed.right,
                                         lastTarget_.motorSpeed.right - dt * maxAccel,
                                         lastTarget_.motorSpeed.right + dt * maxAccel);
    target.motorSpeed.left = std::clamp(target.motorSpeed.left,
                                        lastTarget_.motorSpeed.left - dt * maxAccel,
                                        lastTarget_.motorSpeed.left + dt * maxAccel);
    lastTarget_ = target;


    if (measurements.matchTime > 0.0)
    {
        log("MotionController.commandVelocityRight",target.motorSpeed.right);
        log("MotionController.commandVelocityLeft",target.motorSpeed.left);
        log("MotionController.lockTimeSinceFirstStop",lockTimeSinceFirstStop_);
    }

    return target;
}


void MotionController::changeMotionControllerState()
{

    // in all cases, if new trajectories, reset controller
    // Load new trajectory, if needed.
    newTrajectoryMutex_.lock();
    if (!newTrajectories_.empty())
    {
        // *logger_ << "[MotionController] New trajectories, reset controller state to WAIT_FOR_TRAJECTORY" << std::endl;
        motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;
    }

    if (askedForTrackingStop_)
    {
        askedForTrackingStop_ = false;
        currentTrajectories_.clear();
        motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;
    }

    MotionControllerState nextMotionControllerState = motionControllerState_;

    if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
    {
        lockTimeSinceFirstStop_ = false;
        if (!newTrajectories_.empty())
        {
            // *logger_ << "[MotionController] Start reading trajectories " << std::endl;
            // We have new trajectories, erase the current trajectories and follow the new one.
            currentTrajectories_ = newTrajectories_;
            curvilinearAbscissa_ = 0;
            avoidanceCount_ = 0;
            trajectoryDone_ = false;
            newTrajectories_.clear();
            // *logger_ << "[MotionController] Recieved " << currentTrajectories_.size() << " new trajectories from strategy" << std::endl;

            nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        }
    }
    newTrajectoryMutex_.unlock();

    if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    {
        // Potentially unlock stop if we have moved lone enough.
        if (clampedSlowDownCoeff_ < 0.99)
        {
            fullSpeedStartTime_ = std::chrono::steady_clock::now();
        }
        double const durationSinceFullSpeed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - fullSpeedStartTime_).count() / 1000.0;
        if (durationSinceFullSpeed > 1.0)
            lockTimeSinceFirstStop_ = false;

        // transition to STOP
        if (clampedSlowDownCoeff_ < 1.0e-6)
        {
            if (!lockTimeSinceFirstStop_)
            {
                timeSinceFirstStopped_ = std::chrono::steady_clock::now();
                lockTimeSinceFirstStop_ = true;
            }
            nextMotionControllerState = CONTROLLER_STOP;
        }
        else
        {
            // Load first trajectory, look if we are done following it.
            Trajectory *traj = currentTrajectories_.at(0).get();

            if (curvilinearAbscissa_ > traj->getDuration() || trajectoryDone_)
            {
                currentTrajectories_.erase(currentTrajectories_.begin());
                // If we still have a trajectory after that, immediately switch to the next trajectory.
                if (currentTrajectories_.size() > 0)
                {
                    // Not optimal, could be improved.
                    traj = currentTrajectories_.at(0).get();
                    curvilinearAbscissa_ = 0.0;
                    // *logger_ << "[MotionController] Now tracking: " << traj->description_ << std::endl;
                }
                else
                {
                    *logger_ << "[MotionController] Trajectory tracking performed successfully" << std::endl;
                }
            }

            // If we are more than a specified time after the end of the trajectory, stop it anyway.
            // We hope to have servoed the robot is less than that anyway.
            else if (curvilinearAbscissa_ - trajectoryTimeout_ > traj->getDuration() + TRAJECTORY_TRACKING_TIMEOUT)
            {
                *logger_ << "[MotionController] Timeout on trajectory following" << std::endl;
                currentTrajectories_.erase(currentTrajectories_.begin());
                curvilinearAbscissa_ = 0.;
            }
        }

        // transition to WAIT_FOR_TRAJECTORY
        if (currentTrajectories_.empty())
        {
            nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
        }
    }
    else if (motionControllerState_ == CONTROLLER_STOP)
    {
        // in seconds
        double durationSinceFirstStopped = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timeSinceFirstStopped_).count() / 1000.0;
        double durationSinceLastAvoidance = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timeSinceLastAvoidance_).count() / 1000.0;

        // transition to TRAJECTORY_TRACKING
        if (clampedSlowDownCoeff_ > 0.0 && durationSinceFirstStopped > 0.1)
        {
            // Robot was stopped and is ready to start again.
            // Replan and retry trajectory.
            currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
            curvilinearAbscissa_ = 0;
            *logger_ << "[MotionController] "  << "Continue trajectory; replan" << std::endl;
            nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        }
        else if (durationSinceFirstStopped > 0.5 && durationSinceLastAvoidance > 0.5)
        {
            avoidanceCount_++;
            // transition to WAIT_FOR_TRAJECTORY
            if (avoidanceCount_ > maxAvoidanceAttempts_)
            {
                *logger_ << "[MotionController] " << "Trajectory failed: attempted " << avoidanceCount_ << " avoidance" << std::endl;
                *logger_ << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
                // Failed to perform avoidance.
                // Raise flag and end trajectory following.
                wasTrajectoryFollowingSuccessful_ = false;
                currentTrajectories_.clear();
                nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
            }
            // transition to AVOIDANCE
            else
            {
                *logger_ << "[MotionController] " << "Scheduling avoidance" << std::endl;

                avoidanceComputationMutex_.lock();
                avoidanceComputationScheduled_ = true;
                avoidanceComputationMutex_.unlock();

                timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
                *logger_ << "slowDownCoeff_ : " << slowDownCoeff_ << std::endl;
                *logger_ << "clampedSlowDownCoeff_ : " << clampedSlowDownCoeff_ << std::endl;
                *logger_ << "avoidanceCount_ : " << avoidanceCount_ << std::endl;

                nextMotionControllerState = CONTROLLER_WAIT_FOR_AVOIDANCE;
            }
        }
    }
    else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
    {
        if (!avoidanceComputationScheduled_)
        {
            // transition to  TRAJECTORY_TRACKING
            if (avoidanceComputationResult_.getDuration() > 0)
            {
                *logger_ << "[MotionController] " << "Performing avoidance" << std::endl;

                avoidanceComputationMutex_.lock();
                currentTrajectories_ = avoidanceComputationResult_;
                avoidanceComputationMutex_.unlock();
                curvilinearAbscissa_ = 0.0;

                nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
            }
            // transition to STOP
            else
            {
                *logger_ << "[MotionController] " << "Avoidance failed" << std::endl;

                nextMotionControllerState = CONTROLLER_STOP;
            }
            avoidanceComputationMutex_.lock();
            avoidanceComputationScheduled_ = false;
            avoidanceComputationMutex_.unlock();

        }
    }

    // print and change
    // if (motionControllerState_ != nextMotionControllerState)
    // {
    //     std::string current;
    //     if (motionControllerState_ == CONTROLLER_STOP)
    //     {
    //         current = "STOP";
    //     }
    //     else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    //     {
    //         current = "TRAJECTORY_TRACKING";
    //     }
    //     else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
    //     {
    //         current = "WAIT_FOR_AVOIDANCE";
    //     }
    //     else if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
    //     {
    //         current = "WAIT_FOR_TRAJECTORY";
    //     }

    //     std::string next;
    //     if (nextMotionControllerState == CONTROLLER_STOP)
    //     {
    //         next = "STOP";
    //     }
    //     else if (nextMotionControllerState == CONTROLLER_TRAJECTORY_TRACKING)
    //     {
    //         next = "TRAJECTORY_TRACKING";
    //     }
    //     else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_AVOIDANCE)
    //     {
    //         next = "WAIT_FOR_AVOIDANCE";
    //     }
    //     else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_TRAJECTORY)
    //     {
    //         next = "WAIT_FOR_TRAJECTORY";
    //     }

    //     *logger_ << "[MotionController] State changed: from " << current << " to " << next << std::endl;
    // }

    motionControllerState_ = nextMotionControllerState;
}

DrivetrainTarget MotionController::resolveMotionControllerState(
    DrivetrainMeasurements const &measurements,
    double const &dt,
    bool const &hasMatchStarted
)
{
    DrivetrainTarget target;
    target.motorSpeed.right = 0.0;
    target.motorSpeed.left = 0.0;

    if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        curvilinearAbscissa_ += clampedSlowDownCoeff_ * dt;

        // Servo robot on current trajectory.
        trajectoryDone_ = computeMotorTarget(traj, curvilinearAbscissa_, dt, clampedSlowDownCoeff_, measurements, target);
        // if (trajectoryDone && currentTrajectories_.size() == 1)
        // {
        //     *logger_ << "[MotionController] Trajectory tracking performed successfully" << std::endl;
        //     currentTrajectories_.erase(currentTrajectories_.begin());
        // }
    }

    if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY || motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
        lockTimeSinceFirstStop_ = false;

    return target;
}


bool MotionController::goToRoundedCorners(std::vector<RobotPosition> const& positions,
                                          double radius,
                                          double transitionVelocityFactor,
                                          tf const& flags)
{
    std::vector<RobotPosition> trajPositions = positions;
    trajPositions.insert(trajPositions.begin(), getCurrentPosition());

    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
        getCurrentTrajectoryParameters(),
        trajPositions,
        radius,
        transitionVelocityFactor,
        flags
    );
    setTrajectoryToFollow(traj);
    if (!(flags & tf::NO_WAIT_FOR_END))
        return waitForTrajectoryFinished();
    return true;
}


bool MotionController::goToStraightLine(RobotPosition const& position,
                                        double const& speedFactor,
                                        tf const& flags)
{
    // Don't go faster than maximum
    double const clampedFactor = std::clamp(speedFactor, 0.0, 1.75);

    miam::trajectory::TrajectoryConfig conf = getCurrentTrajectoryParameters();
    conf.maxWheelVelocity *= clampedFactor;
    conf.maxWheelAcceleration *= clampedFactor;
    RobotPosition currentPosition = getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
        conf,
        currentPosition, // start
        position, // end
        0.0, // no velocity at end point
        flags
    );
    setTrajectoryToFollow(traj);
    if (!(flags & tf::NO_WAIT_FOR_END))
        return waitForTrajectoryFinished();
    return true;
}

bool MotionController::goStraight(double const& distance, double const& speedFactor, tf const& flags)
{
    RobotPosition targetPosition = getCurrentPosition();
    double const angle = targetPosition.theta + (distance < 0 ? M_PI : 0);
    targetPosition.x += std::cos(angle) * std::abs(distance);
    targetPosition.y += std::sin(angle) * std::abs(distance);
    tf flg= flags;
    if (distance < 0)
        flg = static_cast<tf>(flg | tf::BACKWARD);
    return goToStraightLine(targetPosition, speedFactor, flg);
}

bool MotionController::pointTurn(double const& angle, double const& speedFactor, tf const& flags)
{
    // Don't go faster than maximum
    double const clampedFactor = std::clamp(speedFactor, 0.0, 1.75);

    miam::trajectory::TrajectoryConfig conf = getCurrentTrajectoryParameters();
    conf.maxWheelVelocity *= clampedFactor;
    conf.maxWheelAcceleration *= clampedFactor;
    RobotPosition currentPosition = getCurrentPosition();
    double const endAngle = currentPosition.theta + angle;

    TrajectoryVector traj;
    traj.push_back(std::shared_ptr<Trajectory>(
        new PointTurn(conf, currentPosition, endAngle)));

    setTrajectoryToFollow(traj);
    if (!(flags & tf::NO_WAIT_FOR_END))
        return waitForTrajectoryFinished();
    return true;
}

TrajectoryConfig MotionController::getCurrentTrajectoryParameters()
{
    TrajectoryConfig conf = robotParams_.getTrajConf();
    if (gameState_.isBackClawFull || gameState_.isFrontClawFull)
    {
        conf.maxWheelVelocity *= 0.6;
        conf.maxWheelAcceleration *= 0.6;
    }
    return conf;
}