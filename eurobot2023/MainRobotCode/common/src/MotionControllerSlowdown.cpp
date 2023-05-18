#include <common/MotionController.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/TextLogger.h>

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

        // If the current trajectory is a point turn, then do not slow down or stop
        if (currentTrajectories_.size() > 0 && std::abs(currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity) > 1e-6)
        {

            double x_max = detection::x_max;
            double y_max = detection::y_max;
            double xfar_max = detection::xfar_max;
            double yfar_max = detection::yfar_max;

            // If currently avoiding, then use less strict thresholds
            if (currentTrajectories_.front()->isAvoidanceTrajectory_)
            {
                x_max = detection::x_max_avoidance;
                y_max = detection::y_max_avoidance;
                xfar_max = detection::xfar_max_avoidance;
                yfar_max = detection::yfar_max_avoidance;
            }
            else if (!currentTrajectories_.front()->isAvoidanceEnabled())
            {
                x_max = detection::x_max_ending;
                y_max = 0.0;
                xfar_max = 0.0;
                yfar_max = 0.0;
            }

            if (x > 0)
            {
                if (x < x_max)
                {
                    if (std::abs(y) < y_max)
                    {
                        // Stop robot
                        coeff = 0.0;
                        is_robot_stopped = true;
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