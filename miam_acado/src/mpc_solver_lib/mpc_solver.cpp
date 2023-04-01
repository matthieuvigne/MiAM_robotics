#include <mpc_solver.hpp>

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#include<iostream>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include "mpc_parameterization.hpp"
#include "gnuplot-iostream.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/*
 * Parameters with which the custom solver was compiled
 */
#define DELTA_T    MPC_DELTA_T
#define HORIZON_T   DELTA_T * N

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace miam;
using namespace std;

std::vector<TrajectoryPoint> solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints
) 
{
    trajectory::TrajectoryVector traj;

    // parameterize solver
    miam::trajectory::TrajectoryConfig c = getMPCTrajectoryConfig();
    miam::trajectory::TrajectoryConfig cplan = getMPCTrajectoryConfig();
    cplan.maxWheelVelocity *= 0.8; // give 20% overhead to the controller

    // create trajectory interpolating waypoints
    traj = computeTrajectoryBasicPath(cplan, waypoints);

    /* Initialize the solver. */
    cout << "Initializing solver" << endl;
    acado_initializeSolver(); 

    cout << "N: " << N << endl;
    cout << "ACADO_N: " << ACADO_N << endl;
    cout << "MPC_N_TIME_INTERVALS: " << MPC_N_TIME_INTERVALS << endl;

    miam::trajectory::TrajectoryPoint tp = getCurrentPointBasicPath(traj, 0.0);

    acadoVariables.x0[0] = tp.position.x / 1000.0;
    acadoVariables.x0[1] = tp.position.y / 1000.0;
    acadoVariables.x0[2] = tp.position.theta;
    acadoVariables.x0[3] = tp.linearVelocity / 1000;
    acadoVariables.x0[4] = tp.angularVelocity;

    // perturbation
    float perturbation_scale = 0.0 / 1000.0;
    float perturbation_scale_angle = 0.0;

    for (int indice = 0; indice < N+1; indice++) {

        cout << indice << endl;

        real_t t = indice * DELTA_T ; 

        if (t > getDurationBasicPath(traj)) {
            t = getDurationBasicPath(traj);
        }

        tp = getCurrentPointBasicPath(traj, t);
        std::cout << "true: " << "t=" << t << " --- " << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;

        /* Initialize the states. */        
        acadoVariables.x[ indice * NX ] = tp.position.x / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 1] = tp.position.y / 1000.0  + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 2] = tp.position.theta + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;
    
        /* Initialize the controls. */
        acadoVariables.x[ indice * NX + 3] = tp.linearVelocity / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 4] = tp.angularVelocity + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;

        if (indice < N) {
            acadoVariables.y[ indice * NY ] = tp.position.x / 1000.0;
            acadoVariables.y[ indice * NY + 1] = tp.position.y / 1000.0;
            acadoVariables.y[ indice * NY + 2] = tp.position.theta;
            acadoVariables.y[ indice * NY + 3] = tp.linearVelocity / 1000.0;
            acadoVariables.y[ indice * NY + 4] = tp.angularVelocity ;
        }
    }

    acadoVariables.yN[0] = tp.position.x / 1000.0;
    acadoVariables.yN[1] = tp.position.y / 1000.0;
    acadoVariables.yN[2] = tp.position.theta;
    acadoVariables.yN[3] = tp.linearVelocity / 1000.0;
    acadoVariables.yN[4] = tp.angularVelocity;

    /* Some temporary variables. */
    acado_timer t;

    /* Get the time before start of the loop. */
    acado_tic( &t );

    /* Prepare step */
    acado_preparationStep();

    /* Perform the feedback step. */
    acado_feedbackStep( );

    /* Apply the new control immediately to the process, first NU components. */

    if( VERBOSE ) printf("\t KKT Tolerance = %.3e\n", acado_getKKT() );

    /* Read the elapsed time. */
    real_t te = acado_toc( &t );

    if( VERBOSE ) printf("Average time of one real-time iteration:   %.3g microseconds\n", 1e6 * te);


    std::vector<TrajectoryPoint> outputPoints;
    for (int indice = 0; indice < N+1; indice++) {

        real_t t = indice * DELTA_T;

        if (t > getDurationBasicPath(traj)) {
            t = getDurationBasicPath(traj);
        }

        tp = getCurrentPointBasicPath(traj, t);
        std::cout << "solved: " << "t=" << t << " --- " << 
            tp.position.x << "\t" << 
            tp.position.y << "\t" << 
            tp.position.theta << "\t" << 
            tp.linearVelocity << "\t" << 
            tp.angularVelocity << std::endl;


        outputPoints.push_back(tp);

    }

    return outputPoints;
}
