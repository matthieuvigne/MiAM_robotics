#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/TextLogger.h>

double MotionController::computeObstacleAvoidanceSlowdown(std::deque<DetectedRobot> const &detectedRobots, bool const &hasMatchStarted)
{

    double coeff = 1.0;

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

        // If the current trajectory is a point turn, then do not slow down or stop
        if (currentTrajectories_.size() > 0 && std::abs(currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity) > 1e-6)
        {

            double x_max = detection::x_max;
            double y_max = detection::y_max;
            double xfar_max = detection::xfar_max;
            double yfar_max = detection::yfar_max;

            // in low avoidance if norm from current position is less than a value
            bool isInLowAvoidance = lowAvoidanceZoneEnabled_ &&
                ((getCurrentPosition() - std::get<0>(lowAvoidanceZone_)).norm() < std::get<1>(lowAvoidanceZone_));

            // if (isInLowAvoidance)
            //     std::cout << "Low avoidance" << std::endl;

            // If currently avoiding, then use less strict thresholds
            if (isInLowAvoidance || !currentTrajectories_.front()->isAvoidanceEnabled())
            {
                x_max = detection::x_max_ending;
                y_max = detection::y_max_ending;
                xfar_max = detection::xfar_max_ending;
                yfar_max = detection::yfar_max_ending;
            }
            else if (currentTrajectories_.front()->isAvoidanceTrajectory_)
            {
                x_max = detection::x_max_avoidance;
                y_max = detection::y_max_avoidance;
                xfar_max = detection::xfar_max_avoidance;
                yfar_max = detection::yfar_max_avoidance;
            }

            if (x > 0)
            {
                if (x < x_max)
                {
                    if (std::abs(y) < y_max)
                    {
                        // Stop robot
                        coeff = 0.0;
                        detected_point = point;
                    }
                }
                else if (x < xfar_max)
                {
                    double maximumY = y_max + (x - x_max) / (xfar_max - x_max) * (yfar_max - y_max);
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
    }

    return coeff;
}



double MotionController::computeObstacleAvoidanceSlowdownAnticipateTrajectory(std::deque<DetectedRobot> const& detectedRobots, bool const& hasMatchStarted)
{
    // If no trajectory then don't slow down
    if (currentTrajectories_.empty())
        return 1.0;

    double minSlowDown = 0.4;
    double maxSlowDown = 0.8;

    double dtTimeHorizon = 0.1;
    double minTimeHorizon = 0.5; // Any obstacle crossed below this time horizon will trigger stop
    double maxTimeHorizon = 3.0; // Time horizon to check ; slowdown will be maxSlowDown
    // Between these values, linear interpolation will be applied

    // double obstacleRadius = detection::mpc_obstacle_size;

    double coeff = 1.0;

    // // Create a list of detected robots in table coordinates
    // std::vector<RobotPosition > detectedRobotsTableCoordinates;
    // for (auto& robot : detectedRobots)
    // {
    //     // Get the Lidar Point, symeterize it if needed and check its projection
    //     LidarPoint const point = this->isPlayingRightSide_
    //                                 ? LidarPoint(robot.point.r, -robot.point.theta)
    //                                 : LidarPoint(robot.point.r, robot.point.theta);

    //     if (this->isLidarPointWithinTable(point))
    //     {
    //         detectedRobotsTableCoordinates.push_back(lidarPointToRobotPosition(point));
    //     }
    // }

    TrajectoryPoint futureTrajectoryPoint;
    for (double timeHorizon = 0.0; timeHorizon <= maxTimeHorizon; timeHorizon += dtTimeHorizon)
    {
        // Get the future (not slowed down) position
        futureTrajectoryPoint = currentTrajectories_.getCurrentPoint(curvilinearAbscissa_ + timeHorizon);

        // double minDistanceToObstacle = 1e6;
        for (auto& obstacle : detectedObstacles_)
        {
            // If we detect a future collision
            if ((futureTrajectoryPoint.position - std::get<0>(obstacle)).norm() < std::get<1>(obstacle))
            {
                // If we are going away from the obstacle, authorize the move
                RobotPosition translatedObstacle = std::get<0>(obstacle) - futureTrajectoryPoint.position;
                RobotPosition obstacleInRobotReferential;
                obstacleInRobotReferential.x = translatedObstacle.x * std::cos(-futureTrajectoryPoint.position.theta) 
                    - translatedObstacle.y * std::sin(-futureTrajectoryPoint.position.theta);
                obstacleInRobotReferential.y = translatedObstacle.x * std::sin(-futureTrajectoryPoint.position.theta) 
                    + translatedObstacle.y * std::cos(-futureTrajectoryPoint.position.theta);
                
                // The obstacle is in front of the robot
                if (futureTrajectoryPoint.linearVelocity * obstacleInRobotReferential.x > 0)
                {
                    // If collision is imminent, stop
                    if (timeHorizon < minTimeHorizon)
                    {
                        coeff = 0.0;
                    }
                    else
                    {
                        coeff = std::min(coeff, minSlowDown + (timeHorizon - minTimeHorizon) / (maxTimeHorizon - minTimeHorizon) * (maxSlowDown - minSlowDown));
                    }
                }
            }
        }
    }

    if (coeff < 1.0)
        std::cout << "SlowDown: " << coeff << std::endl;

    return coeff;
}