#include <miam_utils/trajectory/Utilities.h>

#ifndef MOTION_PLANNING_H
     #define MOTION_PLANNING_H

using namespace miam::trajectory;
class MotionPlanning{

public:
    // std::vector<points> planif_trajectory_highLevel(RobotPosition start, RobotPosition end, Map m)
    // {
    //     // Compute traj
    // }

    // miam::Trajectory smoothAndSpeedTrajectory(std::vector<points>)
    // {
    //     return smoothTraj;
    // }

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

    // void updateMap()
    // {

    // }
};

#endif