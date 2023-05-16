#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/TextLogger.h>


void MotionController::setAvoidanceMode(AvoidanceMode avoidanceMode)
{
    avoidanceMode_ = avoidanceMode;
}

bool MotionController::performAvoidance()
{
    RobotPosition targetPosition = currentTargetEndPosition_;
    bool forward = currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity >= 0;
    std::vector<Obstacle> detectedObstacles = getDetectedObstacles();

    if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_OFF)
    {
        textlog << "[MotionController] " << "Avoidance disabled" << std::endl;
        return false;
    }

    TrajectoryVector traj;
    if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_BASIC)
        traj = computeBasicAvoidanceTrajectory(targetPosition, detectedObstacles, forward);
    else if (avoidanceMode_ == AvoidanceMode::AVOIDANCE_MPC)
        traj = computeMPCTrajectory(targetPosition, detectedObstacles, forward, true);

    if (traj.getDuration() > 0)
    {
        currentTrajectories_.clear();
        currentTrajectories_ = traj;
    }

    return traj.getDuration() > 0;

    // // if trajectory is tagged as needing replanning, do that
    // if (!currentTrajectories_.empty() && currentTrajectories_.front()->needReplanning_)
    // {
    //     coeff = 0.0;
    //     is_robot_stopped = true;

    //     RobotPosition targetPosition = currentTrajectories_.back()->getEndPoint().position;

    //     TrajectoryVector traj = computeMPCTrajectory(targetPosition, getDetectedObstacles(), forward);
    //     if (traj.getDuration() > 0 & !traj.front()->needReplanning_) {
    //         textlog << "[MotionController] " << "Setting avoidance trajectory" << std::endl;
    //         currentTrajectories_.clear();
    //         currentTrajectories_ = traj;
    //     }
    //     else
    //     {
    //         textlog << "[MotionController] " << "Replanning failed" << std::endl;
    //         avoidanceCount_++;
    //         if (avoidanceCount_ > maxAvoidanceAttempts_)
    //         {
    //             // Failed to perform avoidance.
    //             // Raise flag and end trajectory following.
    //             wasTrajectoryFollowingSuccessful_ = false;
    //             currentTrajectories_.clear();
    //             textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
    //             avoidanceCount_ = 0;
    //         }
    //     }
    // }

    // // if robot was stopped for long enough do avoidance
    // if (is_robot_stopped)
    // {
    //     numStopIters_++;
    //     if (numStopIters_ < minStopIters_)
    //     {
    //         // Not enough time spend, just slow down.
    //         coeff = 0.2;
    //     }
    //     else if (numStopIters_ > maxStopIters_)
    //     {
    //         // if stopped for a long time, try to perform avoidance
    //         avoidanceCount_++;
    //         textlog << "[MotionController] " << "avoidanceCount_ " << avoidanceCount_ << std::endl;

    //         // if many avoidance was tried before or the trajectory was tagged not to trigger avoidance
    //         if (avoidanceCount_ > maxAvoidanceAttempts_ || !currentTrajectories_.front()->isAvoidanceEnabled())
    //         {
    //             // then declare trajectory failed
    //             if (!currentTrajectories_.front()->isAvoidanceEnabled())
    //             {
    //                 textlog << "[MotionController] " << ">> MotionControllerAvoidance : Avoidance is disabled on this trajectory" << std::endl;
    //             }
    //             // Failed to perform avoidance.
    //             // Raise flag and end trajectory following.
    //             wasTrajectoryFollowingSuccessful_ = false;
    //             currentTrajectories_.clear();
    //             textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
    //             avoidanceCount_ = 0;
    //         }
    //         else
    //         {
    //             // Proceed avoidance

    //             // try to compute a basic avoidance path and follow if succeeded
    //             RobotPosition targetPosition = currentTrajectories_.back()->getEndPoint().position;

    //             TrajectoryVector traj = computeMPCTrajectory(targetPosition, getDetectedObstacles(), forward);
    //             if (traj.getDuration() > 0) {
    //                 textlog << "[MotionController] " << "Setting avoidance trajectory" << std::endl;
    //                 currentTrajectories_.clear();
    //                 currentTrajectories_ = traj;
    //                 coeff = 1.0;
    //                 numStopIters_ = 0;
    //             }

    //         }

    //     }
    // }













    // double coeff = 1.0;
    // bool is_robot_stopped = false;

    // LidarPoint detected_point;
    // detected_point.r = 1e6;
    // detected_point.theta = 0.;

    // // Update trajectory.
    // bool forward = true;
    // if (!currentTrajectories_.empty())
    // {
    //     miam::trajectory::TrajectoryPoint trajectoryPoint = currentTrajectories_.at(0)->getCurrentPoint(curvilinearAbscissa_);
    //     forward = (trajectoryPoint.linearVelocity >= 0);
    // }

    // int nPointsInTable = 0;

    // // if trajectory is tagged as needing replanning, do that
    // if (!currentTrajectories_.empty() && currentTrajectories_.front()->needReplanning_)
    // {
    //     coeff = 0.0;
    //     is_robot_stopped = true;

    //     RobotPosition targetPosition = currentTrajectories_.back()->getEndPoint().position;

    //     TrajectoryVector traj = computeMPCTrajectory(targetPosition, getDetectedObstacles(), forward);
    //     if (traj.getDuration() > 0 & !traj.front()->needReplanning_) {
    //         textlog << "[MotionController] " << "Setting avoidance trajectory" << std::endl;
    //         currentTrajectories_.clear();
    //         currentTrajectories_ = traj;
    //     }
    //     else
    //     {
    //         textlog << "[MotionController] " << "Replanning failed" << std::endl;
    //         avoidanceCount_++;
    //         if (avoidanceCount_ > maxAvoidanceAttempts_)
    //         {
    //             // Failed to perform avoidance.
    //             // Raise flag and end trajectory following.
    //             wasTrajectoryFollowingSuccessful_ = false;
    //             currentTrajectories_.clear();
    //             textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
    //             avoidanceCount_ = 0;
    //         }
    //     }
    // }
    // // else do obstacle detection
    // else
    // {
    //     for (const DetectedRobot &robot : detectedRobots)
    //     {
    //         // Get the Lidar Point, symeterize it if needed and check its projection
    //         LidarPoint const point = this->isPlayingRightSide_
    //                                     ? LidarPoint(robot.point.r, -robot.point.theta)
    //                                     : LidarPoint(robot.point.r, robot.point.theta);

    //         if (!this->isLidarPointWithinTable(point))
    //             continue;
    //         nPointsInTable += 1;

    //         double x = point.r * std::cos(point.theta + (forward ? 0 : M_PI));
    //         double y = point.r * std::sin(point.theta + (forward ? 0 : M_PI));

    //         // If the current trajectory is a point turn, then do not slow down or stop
    //         if (currentTrajectories_.size() > 0 && currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity != 0)
    //         {
    //             // If currently avoiding, then use less strict thresholds
    //             if (currentTrajectories_.front()->isAvoidanceTrajectory_) {
    //                 if (x > 0)
    //                 {
    //                     if (x < detection::x_max_avoidance)
    //                     {
    //                         if (std::abs(y) < detection::y_max_avoidance)
    //                         {
    //                             // Stop robot
    //                             coeff = 0.0;
    //                             is_robot_stopped = true;
    //                             detected_point = point;
    //                         }
    //                         else if (std::abs(y) < detection::y_max_avoidance)
    //                         {
    //                             // Stop robot
    //                             coeff = 0.0;
    //                             is_robot_stopped = true;
    //                             detected_point = point;
    //                         }
    //                     }
    //                     else if (x < detection::xfar_max_avoidance)
    //                     {
    //                         double maximumY = detection::y_max_avoidance + (x - detection::x_max_avoidance) / (detection::xfar_max_avoidance - detection::x_max_avoidance) * (detection::yfar_max_avoidance - detection::y_max_avoidance);
    //                         if (std::abs(y) < maximumY)
    //                         {
    //                             const double current_coeff = std::min(1.0, std::max(0.2, (point.r - detection::r1) / (detection::r2 - detection::r1)));
    //                             if (current_coeff < coeff)
    //                             {
    //                                 coeff = current_coeff;
    //                                 detected_point = point;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             else
    //             {
    //                 if (x > 0)
    //                 {
    //                     if (x < detection::x_max)
    //                     {
    //                         if (std::abs(y) < detection::y_max)
    //                         {
    //                             // Stop robot
    //                             coeff = 0.0;
    //                             is_robot_stopped = true;
    //                             detected_point = point;
    //                         }
    //                     }
    //                     else if (x < detection::xfar_max)
    //                     {
    //                         double maximumY = detection::y_max + (x - detection::x_max) / (detection::xfar_max - detection::x_max) * (detection::yfar_max - detection::y_max);
    //                         if (std::abs(y) < maximumY)
    //                         {
    //                             const double current_coeff = std::min(1.0, std::max(0.2, (point.r - detection::r1) / (detection::r2 - detection::r1)));
    //                             if (current_coeff < coeff)
    //                             {
    //                                 coeff = current_coeff;
    //                                 detected_point = point;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     if (is_robot_stopped)
    //     {
    //         numStopIters_++;
    //         if (numStopIters_ < minStopIters_)
    //         {
    //             // Not enough time spend, just slow down.
    //             coeff = 0.2;
    //         }
    //         else if (numStopIters_ > maxStopIters_)
    //         {
    //             // if stopped for a long time, try to perform avoidance
    //             avoidanceCount_++;
    //             textlog << "[MotionController] " << "avoidanceCount_ " << avoidanceCount_ << std::endl;

    //             // if many avoidance was tried before or the trajectory was tagged not to trigger avoidance
    //             if (avoidanceCount_ > maxAvoidanceAttempts_ || !currentTrajectories_.front()->isAvoidanceEnabled())
    //             {
    //                 // then declare trajectory failed
    //                 if (!currentTrajectories_.front()->isAvoidanceEnabled())
    //                 {
    //                     textlog << "[MotionController] " << ">> MotionControllerAvoidance : Avoidance is disabled on this trajectory" << std::endl;
    //                 }
    //                 // Failed to perform avoidance.
    //                 // Raise flag and end trajectory following.
    //                 wasTrajectoryFollowingSuccessful_ = false;
    //                 currentTrajectories_.clear();
    //                 textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
    //                 avoidanceCount_ = 0;
    //             }
    //             else
    //             {
    //                 // Proceed avoidance

    //                 // try to compute a basic avoidance path and follow if succeeded
    //                 RobotPosition targetPosition = currentTrajectories_.back()->getEndPoint().position;

    //                 TrajectoryVector traj = computeMPCTrajectory(targetPosition, getDetectedObstacles(), forward);
    //                 if (traj.getDuration() > 0) {
    //                     textlog << "[MotionController] " << "Setting avoidance trajectory" << std::endl;
    //                     currentTrajectories_.clear();
    //                     currentTrajectories_ = traj;
    //                     coeff = 1.0;
    //                     numStopIters_ = 0;
    //                 }

    //             }

    //         }
    //     }
    //     else
    //     {
    //         if (numStopIters_ >= minRestartIters_)
    //         {
    //             // Robot was stopped and is ready to start again.
    //             // Replan and retry trajectory.
    //             if (!currentTrajectories_.empty())
    //             {
    //                 currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
    //                 curvilinearAbscissa_ = 0;
    //                 textlog << "[MotionController] " << "Continue trajectory; replan" << std::endl;
    //             }
    //         }
    //         numStopIters_ = 0;
    //     }
    // }

    // // Before match: just return coeff, don't trigger memory.

    // // FIXME !
    // if (!hasMatchStarted || currentTrajectories_.size() == 0)
    // {
    //     numStopIters_ = 0;
    //     return coeff;
    // }

    // if (numStopIters_ >= minStopIters_ && numStopIters_ < minRestartIters_)
    // {
    //     numStopIters_++;
    //     // Not ready to restart, just stop
    //     return 0.0;
    // }

    // //   textlog << "[MotionController] " << "numStopIters_ " << numStopIters_ << std::endl;
    // //   textlog << "[MotionController] " << "avoidanceCount_ " << avoidanceCount_ << std::endl;

    // return coeff;

    return true;
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
    numStopIters_ = 0;

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


TrajectoryVector MotionController::computeMPCTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles,
    bool forward, bool avoidanceEnabled)
{
    TrajectoryVector traj;
    RobotPosition currentPosition = getCurrentPosition();

    textlog << "[MotionController] " << ">> MotionControllerAvoidance : current position : " << currentPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : target position : " << targetPosition << std::endl;
    textlog << "[MotionController] " << ">> MotionControllerAvoidance : planning MPC" << std::endl;

    if ((currentPosition - targetPosition).norm() < 150)
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
        tc_pt.maxWheelAcceleration *= 0.5;

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
    motionPlanner_->pathPlanner_->resetCollisions();
    for (auto obstacle : detectedObstacles)
    {
        motionPlanner_->pathPlanner_->addCollision(std::get<0>(obstacle), std::get<1>(obstacle));
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
    TrajectoryVector traj2 = motionPlanner_->planMotion(
        newStartPoint,
        targetPosition
    );

    // if motion planning failed, plan a straight line in order to go back anyway
    if (traj2.getDuration() == 0)
    {
        traj2 = computeTrajectoryStraightLineToPoint(
            robotParams_.getTrajConf(),
            currentPosition, // start
            targetPosition,
            !forward
        );

        TrajectoryConfig tc_pt = robotParams_.getTrajConf();
        tc_pt.maxWheelAcceleration *= 0.5;

        // add a point turn to keep the end angle info
        std::shared_ptr<PointTurn > pt_sub_end(
            new PointTurn(tc_pt,
            traj2.getEndPoint().position, targetPosition.theta)
        );
        traj2.push_back(pt_sub_end);

        // tags trajectory to be replanned
        for (auto& subtraj : traj2)
            subtraj->needReplanning_ = true;
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

    return traj;
}