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


TrajectoryConfig getMPCTrajectoryConfig();

TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed = 0.0);

double getDurationBasicPath(TrajectoryVector bp);

TrajectoryPoint getCurrentPointBasicPath(TrajectoryVector bp, double current_time);

#endif