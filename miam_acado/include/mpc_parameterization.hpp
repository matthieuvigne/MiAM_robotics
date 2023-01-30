#ifndef MPC_parameterization
#define MPC_parameterization
    

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#define MPC_DELTA_T 0.1// 100 ms
#define MPC_N_TIME_INTERVALS 20 // 20 discrete time intervals

#define MPC_MU_TRAJ 100 // weight of the trajectory (x, y) in the optimization algorithm
#define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
#define MPC_MU_VLIN 0.01 // weight of the trajectory (v) in the optimization algorithm
#define MPC_MU_VANG 0.01 // weight of the trajectory (w) in the optimization algorithm

#define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

using namespace miam;
using namespace miam::trajectory;

TrajectoryConfig getMPCTrajectoryConfig() {
    TrajectoryConfig c;
    c.maxWheelVelocity = 500;
    c.maxWheelAcceleration = 600;
    c.robotWheelSpacing = 100.5;
    return c;
};

TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed = 0.0)
{
    TrajectoryVector vector;

    for (int i = 0; i < p.size() - 1; i++) {
        RobotPosition p1 = p.at(i);
        RobotPosition p2 = p.at(i+1);

        double starting_speed;
        if (i == 0) {
            starting_speed = initialSpeed;
        } else {
            // starting speed is the ending speed of the preceding trajectory
            // do not take the endpoint or else the speed is null
            starting_speed = vector.back()->getCurrentPoint(vector.back()->getDuration() - 0.001).linearVelocity;
        }

        std::shared_ptr<StraightLine> line(new StraightLine(
            config, p1, p2, 
            starting_speed, 
            config.maxWheelVelocity, false));

        // Go from point to point.
        vector.push_back(std::shared_ptr<Trajectory>(line));
    }

    return vector;
};

double getDurationBasicPath(TrajectoryVector bp) {
    double sum_time = 0;
    for (int i = 0; i < bp.size(); i++) {
        sum_time += bp.at(i)->getDuration();
    }
    return sum_time;
}

TrajectoryPoint getCurrentPointBasicPath(TrajectoryVector bp, double current_time) {
    double sum_time = 0;
    for (int i = 0; i < bp.size(); i++) {
        double traj_time = bp.at(i)->getDuration();
        if (sum_time + traj_time > current_time) {
            return bp.at(i)->getCurrentPoint(current_time - sum_time);
        }
        sum_time += traj_time;
    }
    return bp.back()->getEndPoint();
};

#endif