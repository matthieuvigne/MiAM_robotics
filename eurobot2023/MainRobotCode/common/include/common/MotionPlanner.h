#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/SampledTrajectory.h>

#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H


// The parameters used to generate the MPC code
#define MPC_DELTA_T 0.1// 100 ms
#define MPC_N_TIME_INTERVALS 20 // 20 discrete time intervals

#define MPC_MU_TRAJ 100 // weight of the trajectory (x, y) in the optimization algorithm
#define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
#define MPC_MU_VLIN 0.01 // weight of the trajectory (v) in the optimization algorithm
#define MPC_MU_VANG 0.01 // weight of the trajectory (w) in the optimization algorithm

#define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

using namespace miam;   
using namespace miam::trajectory;
class MotionPlanner{

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

TrajectoryConfig getMPCTrajectoryConfig();

TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed);

double getDurationBasicPath(TrajectoryVector bp);

std::shared_ptr<SampledTrajectory > solveMPCIteration(
    TrajectoryVector reference_trajectory,
    TrajectoryPoint start_position,
    TrajectoryPoint target_position,
    double start_time
);

TrajectoryVector solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints
);


#endif