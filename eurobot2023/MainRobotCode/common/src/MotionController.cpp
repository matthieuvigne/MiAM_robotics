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

    // Create solver thread
    avoidanceComputationScheduled_ = false;
    avoidanceComputationEnded_ = false;

    std::thread stratSolver(&MotionController::loopOnAvoidanceComputation, this);
    pthread_t handle = stratSolver.native_handle();
    createdThreads_.push_back(handle);
    stratSolver.detach();

    motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;
    timeSinceFirstStopped_ = std::chrono::steady_clock::now();
    timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
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
    log("currentVelocityLinear",baseSpeed.linear / dt);
    log("currentVelocityAngular",baseSpeed.angular / dt);

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

    changeMotionControllerState();

    // Compute slowdown
    slowDownCoeff_ = computeObstacleAvoidanceSlowdown(measurements.lidarDetection, hasMatchStarted);
    clampedSlowDownCoeff_ = 1.0;
    clampedSlowDownCoeff_ = std::min(slowDownCoeff_, clampedSlowDownCoeff_ + 0.05);

    target = resolveMotionControllerState(measurements, dt, hasMatchStarted);

    log("motionControllerState", static_cast<int>(motionControllerState_));
    log("detectionCoeff",slowDownCoeff_);
    log("clampedDetectionCoeff", clampedSlowDownCoeff_);
    log("commandVelocityRight",target.motorSpeed[side::RIGHT]);
    log("commandVelocityLeft",target.motorSpeed[side::LEFT]);

    return target;
}


void MotionController::changeMotionControllerState()
{

    MotionControllerState nextMotionControllerState = motionControllerState_;

    // handle changes depending on the current controller state
    if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
    {
        // transition to CONTROLLER_TRAJECTORY_TRACKING
        // Load new trajectory, if needed.
        newTrajectoryMutex_.lock();
        if (!newTrajectories_.empty())
        {
            // We have new trajectories, erase the current trajectories and follow the new one.
            currentTrajectories_ = newTrajectories_;
            curvilinearAbscissa_ = 0;
            avoidanceCount_ = 0;
            newTrajectories_.clear();
            textlog << "[MotionController] Recieved " << currentTrajectories_.size() << " new trajectories from strategy" << std::endl;
            textlog << "[MotionController] Now tracking: " << currentTrajectories_.at(0)->description_ << std::endl;

            nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        }
        newTrajectoryMutex_.unlock();
        if (!currentTrajectories_.empty())
        {
            textlog << "[MotionController] Start reading trajectories " << std::endl;
            nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        }
    }
    else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    {
        // transition to WAIT_FOR_TRAJECTORY
        if (currentTrajectories_.empty())
        {
            nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
        }
        // transition to STOP
        else if (clampedSlowDownCoeff_ < 1.0e-6)
        {
            timeSinceFirstStopped_ = std::chrono::steady_clock::now();

            nextMotionControllerState = CONTROLLER_STOP;
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
            textlog << "[MotionController] "  << "Continue trajectory; replan" << std::endl;
            nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        }
        // transition to WAIT_FOR_TRAJECTORY
        else if (avoidanceCount_ > maxAvoidanceAttempts_)
        {
            textlog << "[MotionController] " << "Trajectory failed: attempted " << avoidanceCount_ << " avoidance" << std::endl;
            textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
            // Failed to perform avoidance.
            // Raise flag and end trajectory following.
            wasTrajectoryFollowingSuccessful_ = false;
            currentTrajectories_.clear();
            curvilinearAbscissa_ = 0.0;
            nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
        }
        // transition to AVOIDANCE
        else if (durationSinceFirstStopped > 1.0 && durationSinceLastAvoidance > 1.0)
        {
            textlog << "[MotionController] " << "Scheduling avoidance" << std::endl;

            avoidanceComputationMutex_.lock();
            avoidanceComputationScheduled_ = true;
            avoidanceComputationEnded_ = false;
            avoidanceComputationMutex_.unlock();

            timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
            avoidanceCount_++;
            std::cout << "slowDownCoeff_ : " << slowDownCoeff_ << std::endl;
            std::cout << "clampedSlowDownCoeff_ : " << clampedSlowDownCoeff_ << std::endl;
            std::cout << "avoidanceCount_ : " << avoidanceCount_ << std::endl;

            nextMotionControllerState = CONTROLLER_WAIT_FOR_AVOIDANCE;
        }
    }
    else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
    {
        if (avoidanceComputationEnded_)
        {
            // transition to  TRAJECTORY_TRACKING
            if (avoidanceComputationResult_.getDuration() > 0)
            {
                textlog << "[MotionController] " << "Performing avoidance" << std::endl;

                currentTrajectories_.clear();
                avoidanceComputationMutex_.lock();
                currentTrajectories_ = avoidanceComputationResult_;
                avoidanceComputationMutex_.unlock();
                curvilinearAbscissa_ = 0.0;

                nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
            }
            // transition to STOP
            else
            {
                textlog << "[MotionController] " << "Avoidance failed" << std::endl;

                nextMotionControllerState = CONTROLLER_STOP;
            }
            avoidanceComputationMutex_.lock();
            avoidanceComputationScheduled_ = false;
            avoidanceComputationEnded_ = false;
            avoidanceComputationMutex_.unlock();

        }
    }

    // print and change
    if (motionControllerState_ != nextMotionControllerState)
    {
        std::string current;
        if (motionControllerState_ == CONTROLLER_STOP)
        {
            current = "STOP";
        }
        else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
        {
            current = "TRAJECTORY_TRACKING";
        }
        else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
        {
            current = "WAIT_FOR_AVOIDANCE";
        }
        else if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
        {
            current = "WAIT_FOR_TRAJECTORY";
        }

        std::string next;
        if (nextMotionControllerState == CONTROLLER_STOP)
        {
            next = "STOP";
        }
        else if (nextMotionControllerState == CONTROLLER_TRAJECTORY_TRACKING)
        {
            next = "TRAJECTORY_TRACKING";
        }
        else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_AVOIDANCE)
        {
            next = "WAIT_FOR_AVOIDANCE";
        }
        else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_TRAJECTORY)
        {
            next = "WAIT_FOR_TRAJECTORY";
        }

        textlog << "[MotionController] State changed: from " << current << " to " << next << std::endl;
    }

    motionControllerState_ = nextMotionControllerState;
}

DrivetrainTarget MotionController::resolveMotionControllerState(
    DrivetrainMeasurements const &measurements,
    double const &dt,
    bool const &hasMatchStarted
)
{
    DrivetrainTarget target;
    target.motorSpeed[side::RIGHT] = 0.0;
    target.motorSpeed[side::LEFT] = 0.0;

    if (!hasMatchStarted)
    {
        target.motorSpeed[side::RIGHT] = 0.0;
        target.motorSpeed[side::LEFT] = 0.0;
        PIDLinear_.resetIntegral(0.0);
        PIDAngular_.resetIntegral(0.0);
    }
    else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        curvilinearAbscissa_ += clampedSlowDownCoeff_ * dt;

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
            bool trajectoryDone = computeMotorTarget(traj, curvilinearAbscissa_, dt, clampedSlowDownCoeff_, measurements, target);
            if (trajectoryDone && currentTrajectories_.size() == 1)
            {
                textlog << "[MotionController] Trajectory tracking performed successfully" << std::endl;
                currentTrajectories_.erase(currentTrajectories_.begin());
            }
        }
    }

    return target;
}