#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>


void MotionController::setAvoidanceMode(AvoidanceMode avoidanceMode)
{
    avoidanceMode_ = avoidanceMode;
}

TrajectoryVector MotionController::performAvoidance()
{
    TrajectoryVector traj;

    if (!currentTrajectories_.front()->isAvoidanceEnabled())
    {
        *logger_ << "[MotionController] performAvoidance: Avoidance is disabled for this traj" << std::endl;
        return traj;
    }
    else if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_OFF)
    {
        *logger_ << "[MotionController] performAvoidance: Avoidance disabled" << std::endl;
        return traj;
    }

    RobotPosition targetPosition = currentTargetEndPosition_;
    bool forward = true;
    if (!currentTrajectories_.empty())
    {
        forward = currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity >= 0;
    }

    std::vector<Obstacle> detectedObstacles = getDetectedObstacles();
    *logger_ << "[MotionController] performAvoidance: Obstacles detected: " << detectedObstacles.size() << std::endl;
    // if (avoidPersistentObstacles_)
    // {
    //     *logger_ << "[MotionController] performAvoidance: Avoiding also persistent obstacles" << std::endl;
    //     std::vector<Obstacle> persistentObstacles = getPersistentObstacles();
    //     copy(persistentObstacles.begin(), persistentObstacles.end(), back_inserter(detectedObstacles));
    //     *logger_ << "[MotionController] performAvoidance: New obstacles count: " << detectedObstacles.size() << std::endl;
    // }
    tf const flags = (forward ? tf::DEFAULT : tf::BACKWARD);
    if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_BASIC)
        traj = computeBasicAvoidanceTrajectory(targetPosition, detectedObstacles, flags);
    else if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_MPC)
    {
        // The more avoidance fails, the more we go backward.
        double const backwardMargin = std::min(avoidanceCount_ * 75, 150);
        traj = computeMPCTrajectory(targetPosition, detectedObstacles, flags, backwardMargin);
    }

    for (auto& subtraj : traj)
        subtraj->isAvoidanceTrajectory_ = true;

    return traj;
}


void MotionController::loopOnAvoidanceComputation()
{
    while (true)
    {
        avoidanceComputationMutex_.lock();
        bool needRecomputing = avoidanceComputationScheduled_;
        avoidanceComputationMutex_.unlock();

        if (needRecomputing)
        {
            *logger_ << "[MotionControllerAvoidance] recomputing avoidace" << std::endl;
            TrajectoryVector res = performAvoidance();
            *logger_ << "[MotionControllerAvoidance] loopOnAvoidanceComputation ended" << std::endl;
            avoidanceComputationMutex_.lock();
            avoidanceComputationResult_.clear();
            avoidanceComputationResult_ = res;
            avoidanceComputationScheduled_ = false;
            avoidanceComputationMutex_.unlock();
        }

        usleep(50000);
    }
}



TrajectoryVector MotionController::computeBasicAvoidanceTrajectory(RobotPosition targetPosition,
                                                                   std::vector<Obstacle> detectedObstacles,
                                                                   tf const& flags)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    *logger_ << "[MotionController] >> MotionControllerAvoidance : current position : " << currentPosition << std::endl;
    *logger_ << "[MotionController] >> MotionControllerAvoidance : target position : " << targetPosition << std::endl;

    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    // Set the new trajectory
    *logger_ << "[MotionController] >> MotionControllerAvoidance : Triggering basic avoidance" << std::endl;

    RobotPosition currentPositionModified = currentPosition;

    // if going backwards, left and right are inverted
    if (flags & tf::BACKWARD)
    {
        currentPositionModified.theta = moduloTwoPi(currentPositionModified.theta + M_PI);
    }

    // Compute points to the left of the robot
    RobotPosition left_point(currentPositionModified);
    // 40 cm to the left
    left_point.x -= 400 * sin(left_point.theta);
    left_point.y += 400 * cos(left_point.theta);
    // further 40 cm
    RobotPosition left_point_further(left_point);
    left_point_further.x += 400 * cos(left_point.theta);
    left_point_further.y += 400 * sin(left_point.theta);

    // Compute points to the right of the robot
    RobotPosition right_point(currentPositionModified);
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

        *logger_ << "[MotionController] >> MotionControllerAvoidance : trying to go left" << std::endl;
        *logger_ << "[MotionController] >> MotionControllerAvoidance : waypoint : " << left_point << std::endl;

        positions.clear();
        positions.push_back(currentPosition);
        positions.push_back(left_point);
        positions.push_back(left_point_further);
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3, flags);
    }
    else
    {
        // Attempt right
        if (right_point.x < table_dimensions::table_max_x and right_point.x > table_dimensions::table_min_x and right_point.y < table_dimensions::table_max_y and right_point.y > table_dimensions::table_min_y)
        {

            *logger_ << "[MotionController] >> MotionControllerAvoidance : trying to go right" << std::endl;
            *logger_ << "[MotionController] >> MotionControllerAvoidance : waypoint : " << right_point << std::endl;

            positions.clear();
            positions.push_back(currentPosition);
            positions.push_back(right_point);
            positions.push_back(right_point_further);
            positions.push_back(targetPosition);
            traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3, flags);
        }
    }

    return traj;
}


TrajectoryVector MotionController::computeMPCTrajectory(
    RobotPosition const targetPosition,
    std::vector<Obstacle> const detectedObstacles,
    tf const& flags,
    double const initialBackwardMotionMargin)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    *logger_ << "[MotionController] >> MotionController : planning MPC from " << currentPosition << " to " << targetPosition << std::endl;

    if ((currentPosition - targetPosition).norm() < 400)
    {
        *logger_ << "[MotionController] Target too close, doing a straight line to point" << std::endl;

        return computeTrajectoryStraightLineToPoint(
            robotParams_.getTrajConf(),
            currentPosition, // start
            targetPosition,
            0.0,
            flags);
    }


    // Only one thread may perform planning at a given time.
    motionPlanningMutex_.lock();
    double minDistanceToObstacle = 10000;
    double obstacleRadiusMargin = detection::mpc_obstacle_margin; // add safety distance to obstacle to avoid crossing it

    // update obstacle map
    motionPlanner_.pathPlanner_.resetCollisions();
    for (auto const& obstacle : detectedObstacles)
    {
        motionPlanner_.pathPlanner_.addCollision(std::get<0>(obstacle), std::get<1>(obstacle) + obstacleRadiusMargin);
        minDistanceToObstacle = std::min(minDistanceToObstacle, (std::get<0>(obstacle) - currentPosition).norm());
    }

    *logger_ << "[MotionController] >> Nearest obstacle at " << minDistanceToObstacle << " mm" << std::endl;

    // Go back from the obtacle if needed
    double distanceToGoBack = std::max(
        0.0,
        detection::x_max_avoidance + initialBackwardMotionMargin - minDistanceToObstacle);

    RobotPosition newStartPoint = currentPosition;

    if ((distanceToGoBack > 0))
    {
        // Determine if it's better to go backward or forward, depending on obstacle position.
        RobotPosition forwardPosition(distanceToGoBack, 0.0, 0.0);
        forwardPosition = currentPosition + forwardPosition.rotate(currentPosition.theta);

        RobotPosition backwardPosition(-distanceToGoBack, 0.0, 0.0);
        backwardPosition = currentPosition + backwardPosition.rotate(currentPosition.theta);

        double forwardDistance = 1.0e5;
        double backwardDistance = 1.0e5;
        for (auto const& obstacle : detectedObstacles)
        {
            forwardDistance = std::min(forwardDistance, (std::get<0>(obstacle) - forwardPosition).norm());
            backwardDistance = std::min(backwardDistance, (std::get<0>(obstacle) - backwardPosition).norm());
        }

        if (backwardDistance > forwardDistance)
        {
            *logger_ << "[MotionController] Obstacle too close: moving back " << distanceToGoBack << " mm" << std::endl;
            traj =  traj + miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                -distanceToGoBack
            );
        }
        else
        {
            *logger_ << "[MotionController] Obstacle too close: moving fprward " << distanceToGoBack << " mm" << std::endl;
            traj = traj + miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                distanceToGoBack
            );
        }

        newStartPoint = traj.getEndPoint().position;
    }

    // plan motion
    TrajectoryVector mpcTrajectory = motionPlanner_.planMotion(
        newStartPoint,
        targetPosition,
        flags
    );
    motionPlanningMutex_.unlock();

    if (mpcTrajectory.empty())
    {
        *logger_ << "[MotionController] Avoidance path planning failed" << std::endl;
        return TrajectoryVector();
    }
    else
    {
        *logger_ << "[MotionController] MPC traj planned from " << mpcTrajectory.getCurrentPoint(0.0) << std::endl;
        *logger_ << "[MotionController] to " << mpcTrajectory.getEndPoint() << std::endl;
    }

    return traj + mpcTrajectory;
}