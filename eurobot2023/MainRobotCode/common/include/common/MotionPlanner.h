#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/PathPlanner.h>

#include <common/RobotInterface.h>

#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H

using namespace miam;   
using namespace miam::trajectory;
class MotionPlanner{

    public:

        MotionPlanner(RobotInterface* robot);
        RobotInterface *robot_;

        TrajectoryVector planMotion(
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition);


        TrajectoryVector computeTraj(TrajectoryConfig const& config, RobotPosition start, RobotPosition end)
        {
            return computeTrajectoryStraightLineToPoint(config, start, end);
        }

        double computeMotionTime(TrajectoryConfig const& config, RobotPosition start, RobotPosition end) {

            double duration = 0;

            for (auto v : computeTraj(config, start, end)) {
                duration += v->getDuration();
            }
            return duration;
        }
    
        PathPlanner* pathPlanner_;

};

TrajectoryVector solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints
);

TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed);

TrajectoryConfig getMPCTrajectoryConfig();

std::shared_ptr<SampledTrajectory > solveMPCIteration(
    TrajectoryVector reference_trajectory,
    TrajectoryPoint start_position,
    TrajectoryPoint target_position,
    double start_time
);


#endif