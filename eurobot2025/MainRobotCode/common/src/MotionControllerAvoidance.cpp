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

        usleep(5000);
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
    // Only one thread may perform planning at a given time.
    std::lock_guard lock(motionPlanningMutex_);
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    *logger_ << "[MotionController] Trying to plan MPC from " << currentPosition << " to " << targetPosition << std::endl;

    // Update obstacle map
    double minDistanceToObstacle = 10000;
    double obstacleRadiusMargin = detection::mpc_obstacle_margin; // add safety distance to obstacle to avoid crossing it
    double minObstacleRadius = 0;

    // Update obstacle map
    motionPlanner_.pathPlanner_.resetCollisions();
    for (auto const& obstacle : detectedObstacles)
    {
        motionPlanner_.pathPlanner_.addCollision(std::get<0>(obstacle), std::get<1>(obstacle) + obstacleRadiusMargin);
        double const distance = (std::get<0>(obstacle) - currentPosition).norm();
        if (distance < minDistanceToObstacle)
        {
            minDistanceToObstacle = distance;
            minObstacleRadius = std::get<1>(obstacle);
        }
    }
    if (motionPlanner_.pathPlanner_.isPositionInCollision(targetPosition))
    {
        *logger_ << "[MotionController] End position is in obstacle, cannot plan" << std::endl;
        return traj;
    }

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


    *logger_ << "[MotionController] >> Nearest obstacle at " << minDistanceToObstacle << " mm" << std::endl;

    // Go back from the obtacle if needed - the value to go back is computed
    // to get us out of the avoidance zone:
    //   - minObstacleRadius - minDistanceToObstacle: where the obstacle ends.
    //   - detection::mpc_obstacle_margin : margin in avoidance zone - we take some more room here
    //   - initialBackwardMotionMargin: custom user distance.
    double distanceToGoBack = std::max(
        0.0,
        1.5 * detection::mpc_obstacle_margin + initialBackwardMotionMargin + minObstacleRadius - minDistanceToObstacle);

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

        double sign = (backwardDistance > forwardDistance ? -1 : 1);

        // Clamp point to remain in table, with margin.
        newStartPoint = newStartPoint + RobotPosition(sign * distanceToGoBack, 0.0, 0.0).rotate(newStartPoint.theta);
        newStartPoint.theta = currentPosition.theta;

        newStartPoint.x = std::max(table_dimensions::table_min_x, std::min(table_dimensions::table_max_x, newStartPoint.x));
        newStartPoint.y = std::max(table_dimensions::table_min_y, std::min(table_dimensions::table_max_y, newStartPoint.y));

        distanceToGoBack = sign * (currentPosition - newStartPoint).norm();

        *logger_ << "[MotionController] Obstacle too close: moving " << distanceToGoBack << " mm" << std::endl;
        *logger_ << "[MotionController] Moved back position: " << newStartPoint << std::endl;

        traj = traj + miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                distanceToGoBack
            );

        newStartPoint = traj.getEndPoint().position;
    }

    // At this point, we've tried to move back as much as possible from an obstacle. Nevertheless,
    // we may still be in an obstacle (table limit, robot)...
    // Now we ask the path planner to give us the closest point which is not an obstacle, and we will start planning from there.
    RobotPosition const closestAvailablePosition = motionPlanner_.pathPlanner_.getNearestAvailablePosition(newStartPoint);

    if ((closestAvailablePosition - newStartPoint).norm() > 30)
    {
        *logger_ << "[MotionController] Desired point is not available, using closest point in grid: " << std::endl;
        *logger_ << "[MotionController] " << closestAvailablePosition << " instead of " <<  newStartPoint << std::endl;

        traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(
            robotParams_.getTrajConf(),
            newStartPoint,
            closestAvailablePosition,
            0.0,
            static_cast<tf>(flags | tf::IGNORE_END_ANGLE));
        newStartPoint = traj.getEndPoint().position;
    }

    // plan motion
    TrajectoryVector mpcTrajectory = motionPlanner_.planMotion(
        newStartPoint,
        targetPosition,
        flags
    );

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