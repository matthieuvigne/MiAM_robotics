#include <miam_utils/trajectory/Utilities.h>

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

    TrajectoryVector computeTraj(RobotPosition start, RobotPosition end)
    {
        return computeTrajectoryStraightLineToPoint(start, end);
    }

    double computeMotionTime(RobotPosition start, RobotPosition end) {

        double duration = 0;

        for (auto v : computeTraj(start, end)) {
            duration += v->getDuration();
        }
        return duration;
    }

    // void updateMap()
    // {

    // }
};