#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/TextLogger.h>


void MotionController::setAvoidanceMode(AvoidanceMode avoidanceMode)
{
    avoidanceMode_ = avoidanceMode;
}

TrajectoryVector MotionController::performAvoidance()
{
    TrajectoryVector traj;

    if (!currentTrajectories_.front()->isAvoidanceEnabled())
    {
        textlog << "[MotionController] performAvoidance: Avoidance is disabled for this traj" << std::endl;
        return traj;
    }
    else if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_OFF)
    {
        textlog << "[MotionController] " << "performAvoidance: Avoidance disabled" << std::endl;
        return traj;
    }

    RobotPosition targetPosition = currentTargetEndPosition_;
    bool forward = true;
    if (!currentTrajectories_.empty())
    {
        forward = currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity >= 0;
    }

    std::vector<Obstacle> detectedObstacles = getDetectedObstacles();
    textlog << "[MotionController] " << "performAvoidance: Obstacles detected: " << detectedObstacles.size() << std::endl;
    // if (avoidPersistentObstacles_)
    // {
    //     textlog << "[MotionController] " << "performAvoidance: Avoiding also persistent obstacles" << std::endl;
    //     std::vector<Obstacle> persistentObstacles = getPersistentObstacles();
    //     copy(persistentObstacles.begin(), persistentObstacles.end(), back_inserter(detectedObstacles));
    //     textlog << "[MotionController] " << "performAvoidance: New obstacles count: " << detectedObstacles.size() << std::endl;
    // }

    if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_BASIC)
        traj = computeBasicAvoidanceTrajectory(targetPosition, detectedObstacles, forward);
    else if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_MPC)
        traj = computeMPCAvoidanceTrajectory(targetPosition, detectedObstacles, forward, true);

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
            textlog << "[MotionControllerAvoidance] recomputing avoidace" << std::endl;
            TrajectoryVector res = performAvoidance();
            textlog << "[MotionControllerAvoidance] loopOnAvoidanceComputation ended" << std::endl;
            avoidanceComputationMutex_.lock();
            avoidanceComputationResult_.clear();
            avoidanceComputationResult_ = res;
            avoidanceComputationScheduled_ = false;
            avoidanceComputationMutex_.unlock();
        }

        usleep(50000);
    }
}



TrajectoryVector MotionController::computeBasicAvoidanceTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles, bool forward)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    textlog << "[MotionController] " << ">> MotionControllerAvoidance : current position : " << currentPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : target position : " << targetPosition << std::endl;

    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    // Set the new trajectory
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : Triggering basic avoidance" << std::endl;

    RobotPosition currentPositionModified = currentPosition;

    // if going backwards, left and right are inverted
    if (!forward)
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

        textlog << "[MotionController] " << ">> MotionControllerAvoidance : trying to go left" << std::endl;
        textlog << "[MotionController] " << ">> MotionControllerAvoidance : waypoint : " << left_point << std::endl;

        positions.clear();
        positions.push_back(currentPosition);
        positions.push_back(left_point);
        positions.push_back(left_point_further);
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3, !forward);
    }
    else
    {
        // Attempt right
        if (right_point.x < table_dimensions::table_max_x and right_point.x > table_dimensions::table_min_x and right_point.y < table_dimensions::table_max_y and right_point.y > table_dimensions::table_min_y)
        {

            textlog << "[MotionController] " << ">> MotionControllerAvoidance : trying to go right" << std::endl;
            textlog << "[MotionController] " << ">> MotionControllerAvoidance : waypoint : " << right_point << std::endl;

            positions.clear();
            positions.push_back(currentPosition);
            positions.push_back(right_point);
            positions.push_back(right_point_further);
            positions.push_back(targetPosition);
            traj = miam::trajectory::computeTrajectoryRoundedCorner(robotParams_.getTrajConf(), positions, 200.0, 0.3);
        }
    }

    return traj;
}


TrajectoryVector MotionController::computeMPCAvoidanceTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles,
    bool forward, bool avoidanceEnabled, bool ensureEndAngle)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    textlog << "[MotionController] " << ">> MotionControllerAvoidance : current position : " << currentPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : target position : " << targetPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : planning MPC" << std::endl;

    if ((currentPosition - targetPosition).norm() < 400)
    {
        textlog << "[MotionController] " << "Target too close, doing a straight line to point" << std::endl;

        traj = computeTrajectoryStraightLineToPoint(
            robotParams_.getTrajConf(),
            currentPosition, // start
            targetPosition,
            !forward
        );

        textlog << "[MotionController] " << "computeTrajectoryStraightLineToPoint ends at " << traj.getEndPoint().position << std::endl;

        TrajectoryConfig tc_pt = robotParams_.getTrajConf();
        tc_pt.maxWheelVelocity *= 0.3;
        tc_pt.maxWheelAcceleration *= 0.3;

        // add a point turn to keep the end angle info
        std::shared_ptr<PointTurn > pt_sub_end(
            new PointTurn(tc_pt,
            traj.getEndPoint().position, targetPosition.theta)
        );
        traj.push_back(pt_sub_end);

        textlog << "[MotionController] " << "pt_sub_end ends at " << traj.getEndPoint().position << std::endl;

        return traj;
    }

    double minDistanceToObstacle = 10000;

    // update obstacle map
    motionPlanner_.pathPlanner_.resetCollisions();
    for (auto obstacle : detectedObstacles)
    {
        motionPlanner_.pathPlanner_.addCollision(std::get<0>(obstacle), std::get<1>(obstacle));
        minDistanceToObstacle = std::min(minDistanceToObstacle, (std::get<0>(obstacle) - currentPosition).norm());
    }

    textlog << "[MotionController] " << ">> Nearest obstacle at " << minDistanceToObstacle << " mm" << std::endl;

    // go back from the obstacle (the more counts the more margin)
    double margin = std::min(avoidanceCount_ * 75, 150);
    double distanceToGoBack = std::max(0.0, detection::x_max_avoidance + margin - minDistanceToObstacle);

    TrajectoryVector traj1;
    RobotPosition newStartPoint = currentPosition;

    if ((distanceToGoBack > 0))
    {
        if (forward)
        {
            textlog << "[MotionController] " << ">> Needing to go back " << distanceToGoBack << " mm" << std::endl;
            traj1 = miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                -distanceToGoBack
            );
        }
        else
        {
            textlog << "[MotionController] " << ">> Needing to go forward " << distanceToGoBack << " mm" << std::endl;
            traj1 = miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                distanceToGoBack
            );
        }


        newStartPoint = traj1.getEndPoint().position;

        if (avoidanceEnabled)
        {
            // this is an avoidance traj
            for (auto& subtraj : traj1)
            {
                subtraj->isAvoidanceTrajectory_ = true;
            }
        }


    }

    textlog << "[MotionController] " << "currentPosition: " << currentPosition << std::endl;
    textlog << "[MotionController] " << "newStartPoint: " << newStartPoint << std::endl;

    // plan motion
    newStartPoint.theta = currentPosition.theta;
    TrajectoryVector traj2 = motionPlanner_.planMotion(
        newStartPoint,
        targetPosition,
        ensureEndAngle
    );

    if (traj2.empty())
    {
        textlog << "[MotionController] " << "Avoidance path planning failed" << std::endl;
        return TrajectoryVector();


        // if motion planning failed, plan a straight line in order to go back anyway

        // textlog << "[MotionController] " << "MPC traj failed, planning a dummy straight line" << std::endl;
        // traj2 = computeTrajectoryStraightLineToPoint(
        //     robotParams_.getTrajConf(),
        //     currentPosition, // start
        //     targetPosition,
        //     !forward
        // );

        // TrajectoryConfig tc_pt = robotParams_.getTrajConf();
        // tc_pt.maxWheelVelocity *= 0.3;
        // tc_pt.maxWheelAcceleration *= 0.3;

        // if (ensureEndAngle)
        // {
        //     // add a point turn to keep the end angle info
        //     std::shared_ptr<PointTurn > pt_sub_end(
        //         new PointTurn(tc_pt,
        //         traj2.getEndPoint().position, targetPosition.theta)
        //     );
        //     traj2.push_back(pt_sub_end);
        // }

        // // tags trajectory to be replanned
        // for (auto& subtraj : traj2)
        //     subtraj->needReplanning_ = true;
    }
    else
    {
        textlog << "[MotionController] " << "MPC traj planned from " << traj2.getCurrentPoint(0.0) << std::endl;
        textlog << "[MotionController] " << " to " << traj2.getEndPoint() << std::endl;

        if (avoidanceEnabled)
        {
            // traj is an avoidance traj
            for (auto& subtraj : traj2)
            {
                subtraj->isAvoidanceTrajectory_ = true;
            }
        }
    }

    // if motion planning succeeded, proceed
    if (distanceToGoBack > 10)
    {
        traj.insert( traj.end(), traj1.begin(), traj1.end() );
    }
    traj.insert( traj.end(), traj2.begin(), traj2.end() );

    // textlog << "[MotionControllerAvoidance] MPC trajectory: " << std::endl;
    // for (double t = 0; t <= traj.getDuration(); t+=0.2)
    //     textlog << "[MotionControllerAvoidance] " << traj.getCurrentPoint(t) << std::endl;

    return traj;
}


TrajectoryVector MotionController::computeMPCTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles,
    bool forward)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    textlog << "[MotionController] " << ">> MotionController : current position : " << currentPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionController : target position : " << targetPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionController : planning MPC" << std::endl;

    if ((currentPosition - targetPosition).norm() < 400)
    {
        textlog << "[MotionController] " << "Target too close, doing a straight line to point" << std::endl;

        traj = computeTrajectoryStraightLineToPoint(
            robotParams_.getTrajConf(),
            currentPosition, // start
            targetPosition,
            !forward
        );

        textlog << "[MotionController] " << "computeTrajectoryStraightLineToPoint ends at " << traj.getEndPoint().position << std::endl;

        TrajectoryConfig tc_pt = robotParams_.getTrajConf();
        tc_pt.maxWheelVelocity *= 0.8;
        tc_pt.maxWheelAcceleration *= 0.8;

        // add a point turn to keep the end angle info
        std::shared_ptr<PointTurn > pt_sub_end(
            new PointTurn(tc_pt,
            traj.getEndPoint().position, targetPosition.theta)
        );
        traj.push_back(pt_sub_end);

        textlog << "[MotionController] " << "pt_sub_end ends at " << traj.getEndPoint().position << std::endl;

        return traj;
    }


    double minDistanceToObstacle = 10000;

    // update obstacle map
    motionPlanner_.pathPlanner_.resetCollisions();
    for (auto obstacle : detectedObstacles)
    {
        motionPlanner_.pathPlanner_.addCollision(std::get<0>(obstacle), std::get<1>(obstacle));
        minDistanceToObstacle = std::min(minDistanceToObstacle, (std::get<0>(obstacle) - currentPosition).norm());
    }

    textlog << "[MotionController] " << ">> Nearest obstacle at " << minDistanceToObstacle << " mm" << std::endl;

    // go back from the obstacle (the more counts the more margin)
    double distanceToGoBack = std::max(0.0, detection::x_max  + 150 - minDistanceToObstacle);

    TrajectoryVector traj1;
    RobotPosition newStartPoint = currentPosition;

    if ((distanceToGoBack > 10))
    {
        if (forward)
        {
            textlog << "[MotionController] " << ">> Needing to go back " << distanceToGoBack << " mm" << std::endl;
            traj1 = miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                -distanceToGoBack
            );
        }
        else
        {
            textlog << "[MotionController] " << ">> Needing to go forward " << distanceToGoBack << " mm" << std::endl;
            traj1 = miam::trajectory::computeTrajectoryStraightLine(
                robotParams_.getTrajConf(),
                currentPosition, // start
                distanceToGoBack
            );
        }


        newStartPoint = traj1.getEndPoint().position;
    }
    else
    {
        textlog << "[MotionController] " << ">> Do not need to go back" << std::endl;
    }

    // plan motion
    TrajectoryVector traj2 = motionPlanner_.planMotion(
        newStartPoint,
        targetPosition,
        true
    );

    if (traj2.empty())
    {
        textlog << "[MotionController] " << ">> MPC path planning failed" << std::endl;
        return TrajectoryVector();
    }

    if (!traj1.empty())
    {
        traj.insert( traj.end(), traj1.begin(), traj1.end() );
    }
    traj.insert( traj.end(), traj2.begin(), traj2.end() );

    textlog << "[MotionController] " << "MPC traj planned from " << traj.getCurrentPoint(0.0) << std::endl;
    textlog << "[MotionController] " << " to " << traj.getEndPoint() << std::endl;

    return traj;
}