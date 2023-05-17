#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include <filesystem>
#include <miam_utils/TextLogger.h>

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
    PIDLinear_ = miam::PID(robotParams_.linearKp, robotParams_.linearKd, robotParams_.linearKi, 0.2);
    PIDAngular_ = miam::PID(robotParams_.rotationKp, robotParams_.rotationKd, robotParams_.rotationKi, 0.15);

    // Set MotionPlanner
    motionPlanner_ = new MotionPlanner(robotParameters);

    // Default avoidance mode
    // avoidanceMode_ = AvoidanceMode::AVOIDANCE_BASIC;
    avoidanceMode_ = AvoidanceMode::AVOIDANCE_OFF;
    avoidanceCount_ = 0;
    isStopped_ = false;
}

void MotionController::init(RobotPosition const& startPosition, std::string const& teleplotPrefix)
{
    currentPosition_.set(startPosition);

    // Create logger.
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));

    // Generate unique file ID, because raspberry pi clock is always reset.
    int count = 0;
    std::filesystem::path logDir{"logs/"};
    for (auto& p : std::filesystem::directory_iterator(logDir))
        count++;
    std::string filename = "logs/log" + std::to_string(count) + "_" + std::string(timestamp) + "_" + robotParams_.name + ".hdf5";
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

bool MotionController::setTrajectoryToFollow(TrajectoryVector const &trajectories)
{
    newTrajectoryMutex_.lock();
    newTrajectories_ = trajectories;
    wasTrajectoryFollowingSuccessful_ = true;
    currentTargetEndPosition_ = newTrajectories_.back()->getEndPoint().position;
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

    // Update list of obstacles
    detectedObstaclesMutex_.lock();
    detectedObstacles_.clear();
    filteredDetectedObstacles_.clear();

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
            filteredDetectedObstacles_.push_back(obspos);
            continue;
        }
        filteredDetectedObstacles_.push_back(obspos);
        detectedObstacles_.push_back(std::make_tuple(obspos, detection::mpc_obstacle_size));
        nObstaclesOnTable += 1;
    }
    log("lidarNumberOfObstacles", nObstaclesOnTable);

    // add obstacles
    for (auto obstacle : getPersistentObstacles())
    {
        detectedObstacles_.push_back(obstacle);
    }

    detectedObstaclesMutex_.unlock();

    //////////// Avoidance and slowdown

    // Compute slowdown
    double slowDownCoeff = computeObstacleAvoidanceSlowdown(measurements.lidarDetection, hasMatchStarted);
    log("detectionCoeff",slowDownCoeff);

    if (slowDownCoeff > 0)
    {
        isStopped_ = false;
    }
    else
    {
        // if previously not stopped, then register the time
        if (!isStopped_)
        {
            isStopped_ = true;
            timeSinceFirstStopped_ = std::chrono::steady_clock::now();
            timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
        }
        else
        {
            double durationSinceFirstStopped = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timeSinceFirstStopped_).count();
            double durationSinceLastAvoidance = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timeSinceLastAvoidance_).count();

            // if stopped for longer than a threshold, perform avoidance every 1 second
            if (durationSinceLastAvoidance >= 1000)
            {
                textlog << "[MotionController] " << "Duration since first stopped (ms): " << static_cast<double>(durationSinceFirstStopped) << std::endl;
                textlog << "[MotionController] " << "Duration since last avoidance (ms): " << static_cast<double>(durationSinceLastAvoidance) << std::endl;
                textlog << "[MotionController] " << "Number of avoidance attempts (ms): " << avoidanceCount_ << std::endl;

                // if avoided too much (or stucked for too long TODO), deem trajectory failed
                if (avoidanceCount_ > maxAvoidanceAttempts_)
                {
                    textlog << "[MotionController] " << "Trajectory failed: attempted " << avoidanceCount_ << " avoidance" << std::endl;
                    textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
                    // Failed to perform avoidance.
                    // Raise flag and end trajectory following.
                    wasTrajectoryFollowingSuccessful_ = false;
                    currentTrajectories_.clear();
                    curvilinearAbscissa_ = 0.0;
                }
                else
                {
                    bool avoid = performAvoidance();

                    if (avoid)
                        textlog << "[MotionController] " << "Performing avoidance" << std::endl;
                    else
                        textlog << "[MotionController] " << "Avoidance failed" << std::endl;

                    timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
                    avoidanceCount_++;
                }
            }
        }
    }

    //////////// End of avoidance and slowdown

    // Load new trajectory, if needed.
    newTrajectoryMutex_.lock();
    if (!newTrajectories_.empty())
    {
        // We have new trajectories, erase the current trajectories and follow the new one.
        currentTrajectories_ = newTrajectories_;
        curvilinearAbscissa_ = 0;
        newTrajectories_.clear();
        textlog << "[MotionController] Recieved " << currentTrajectories_.size() << " new trajectories from strategy" << std::endl;
        textlog << "[MotionController] Now tracking: " << currentTrajectories_.at(0)->description_ << std::endl;
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
                textlog << "[MotionController] Now tracking: " << traj->description_ << std::endl;
            }
        }

        // If we are more than a specified time  after the end of the trajectory, stop it anyway.
        // We hope to have servoed the robot is less than that anyway.
        if (curvilinearAbscissa_ - trajectoryTimeout_ > traj->getDuration())
        {
            textlog << "[MotionController] Timeout on trajectory following" << std::endl;
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
                textlog << "[MotionController] Trajectory tracking performed successfully" << std::endl;
                currentTrajectories_.erase(currentTrajectories_.begin());
                target.motorSpeed[0] = 0.0;
                target.motorSpeed[1] = 0.0;
                curvilinearAbscissa_ = 0.;

                // Reset avoidance counter
                avoidanceCount_ = 0;
            }
        }
    }

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

