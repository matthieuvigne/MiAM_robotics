#ifndef MPC_solver
#define MPC_solver


#include <mpc_parameterization.hpp>

std::vector<TrajectoryPoint> solveTrajectoryFromWaypoints(std::vector<RobotPosition> waypoints);

#endif