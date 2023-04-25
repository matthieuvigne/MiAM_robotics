#include "common/MotionPlanner.h"

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/PathPlanner.h>

#include <iostream>
#include <cmath>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

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

#define MPC_OBJECTIVE_TOLERANCE 10 // mm
#define MPC_CONSECUTIVE_POINT_TOLERANCE 1 // mm

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace miam;
using namespace std;

// The parameters used to generate the MPC code
#define MPC_DELTA_T 0.1// 100 ms
#define MPC_N_TIME_INTERVALS 20 // 20 discrete time intervals

#define MPC_MU_TRAJ 100 // weight of the trajectory (x, y) in the optimization algorithm
#define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
#define MPC_MU_VLIN 0.01 // weight of the trajectory (v) in the optimization algorithm
#define MPC_MU_VANG 0.01 // weight of the trajectory (w) in the optimization algorithm

#define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

// ACADO should be inited only once
bool is_acado_inited = false;


TrajectoryConfig getMPCTrajectoryConfig() {
    TrajectoryConfig c;
    c.maxWheelVelocity = 500;
    c.maxWheelAcceleration = 600;
    c.robotWheelSpacing = 100.5;
    return c;
};

MotionPlanner::MotionPlanner(RobotInterface* robot) : robot_(robot)
{
    
}

TrajectoryVector MotionPlanner::planMotion(
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition)
{
    PathPlannerConfig config;
    PathPlanner path_planner(config);
    // path_planner.printMap();
    std::vector<RobotPosition > planned_path = path_planner.planPath(currentPosition, targetPosition);
    path_planner.printMap(planned_path);

    // If path planning failed, return empty traj
    if (planned_path.size() == 0)
    {
        return TrajectoryVector();
    }

    // compute the trajectory from the waypoints
    TrajectoryVector st = solveTrajectoryFromWaypoints(planned_path);
    std::cout << "Solved trajectory: " << std::endl;
    for (double t = 0; t <=st.getDuration(); t += 0.1) {
        std::cout << st.getCurrentPoint(t) << std::endl;
    }

    TrajectoryVector res;

    // at start, perform a point turn to get the right angle
    std::shared_ptr<PointTurn > pt_sub_start(
        new PointTurn(robot_->getParameters().getTrajConf(), 
        currentPosition, st.getCurrentPoint(0.0).position.theta));
    res.push_back(pt_sub_start);

    // insert solved trajectory
    res.insert( res.end(), st.begin(), st.end() );

    // at end, perform a point turn to get the right angle
    std::shared_ptr<PointTurn > pt_sub_end(
        new PointTurn(robot_->getParameters().getTrajConf(), 
        res.getEndPoint().position, targetPosition.theta)
    );
    res.push_back(pt_sub_end);

    return res;
}


TrajectoryVector solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints
) 
{
    trajectory::TrajectoryVector traj;

    // parameterize solver
    miam::trajectory::TrajectoryConfig cplan = getMPCTrajectoryConfig();
    cplan.maxWheelVelocity *= 0.8; // give 20% overhead to the controller

    // create trajectory interpolating waypoints
    traj = computeTrajectoryBasicPath(cplan, waypoints, 0);

    // start of the entire trajectory
    TrajectoryPoint start_position = traj.getCurrentPoint(0.0);
    TrajectoryPoint target_position = traj.getCurrentPoint(traj.getDuration());
    TrajectoryVector res;    

    int nIterMax = ceil(traj.getDuration() / (HORIZON_T - 2 * DELTA_T)) + 1; // enable that many iterations
    int nIter = 0;

    cout << "Duration of the reference path: " << traj.getDuration() << endl;

    // proceed by increments of HORIZON_T - 2 * DELTA_T (not taking the last point since the final
    // constraint might make the solver brutally go towards the final point
    while (nIter < nIterMax)
    {
        std::shared_ptr<SampledTrajectory > st = solveMPCIteration(
            traj,
            start_position,
            target_position,
            nIter * (HORIZON_T - 2 * DELTA_T)
        );
        res.push_back(st);

        if (st->getDuration() < HORIZON_T)
        {
            break;
        }
        start_position = st->getCurrentPoint(HORIZON_T - 2 * DELTA_T);
        nIter++;
    }
    return res;
}


TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed)
{
    TrajectoryVector vector;

    for (long unsigned int i = 0; i < p.size() - 1; i++) {
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

std::shared_ptr<SampledTrajectory > solveMPCIteration(
    TrajectoryVector reference_trajectory,
    TrajectoryPoint start_position,
    TrajectoryPoint target_position,
    double start_time
)
{
    cout << "----------------------------" << endl;
    cout << "Solving starting time: " <<  start_time << endl;
    cout << "Start position: " << start_position << endl;
    cout << "Target position: " << target_position << endl;

    if (!is_acado_inited)
    {
        /* Initialize the solver. */
        cout << "Initializing solver" << endl;
        acado_initializeSolver(); 
        is_acado_inited = true;
    }

    cout << "N: " << N << endl;
    // cout << "ACADO_N: " << ACADO_N << endl;
    // cout << "MPC_N_TIME_INTERVALS: " << MPC_N_TIME_INTERVALS << endl;


    // use this to mitigate the theta modulo two pi instability
    TrajectoryPoint oldtp = start_position;

    acadoVariables.x0[0] = start_position.position.x / 1000.0;
    acadoVariables.x0[1] = start_position.position.y / 1000.0;
    acadoVariables.x0[2] = start_position.position.theta;
    acadoVariables.x0[3] = start_position.linearVelocity / 1000;
    acadoVariables.x0[4] = start_position.angularVelocity;

    // perturbation
    float perturbation_scale = 0.0 / 1000.0;
    float perturbation_scale_angle = 0.0;

    TrajectoryPoint tp;


    for (int indice = 0; indice < N+1; indice++) {

        // cout << indice << endl;

        // start at start_time and iterate over N
        real_t t = indice * DELTA_T + start_time; 

        // if (t > getDuration(reference_trajectory)) {
        //     t = getDuration(reference_trajectory);
        // }

        tp = reference_trajectory.getCurrentPoint(t);

        // oldtp was inited at start_position
        if (abs(tp.position.theta - oldtp.position.theta) > M_PI)
        {
            // cout << "Smoothing angle" << endl;
        }

        // if the gap is > M_PI, then subtract 2 pi 
        if ( tp.position.theta - oldtp.position.theta > M_PI)
        {
            // cout << "Subracting 2pi " << endl;
            tp.position.theta =  tp.position.theta - 2 * M_PI;
            // cout << "New angle: " << tp.position.theta << endl;
        }
        // if the gap is < -M_PI, then add 2 pi 
        if ( tp.position.theta - oldtp.position.theta < -M_PI)
        {
            tp.position.theta = tp.position.theta + 2 * M_PI;
        }
        oldtp = tp;

        // if (VERBOSE)
        // {
        //     std::cout << "true: " << "t=" << t << " --- " << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
        // }
        
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

        real_t t = indice * DELTA_T + start_time;

        tp.position.x      = acadoVariables.x[ indice * NX ] * 1000;
        tp.position.y      = acadoVariables.x[ indice * NX + 1] * 1000;
        tp.position.theta  = acadoVariables.x[ indice * NX + 2];
        tp.linearVelocity  = acadoVariables.x[ indice * NX + 3] * 1000;
        tp.angularVelocity = acadoVariables.x[ indice * NX + 4];

        if (VERBOSE)
        {
            std::cout << "solved: " << "t=" << t << " --- " << 
                tp.position.x << "\t" << 
                tp.position.y << "\t" << 
                tp.position.theta << "\t" << 
                tp.linearVelocity << "\t" << 
                tp.angularVelocity << std::endl;
        }

        // if distance to the final target is < tolerance 
        // or the distance between two consecutive points is < tolerance, then stop


        if (outputPoints.size() > 0)
        {
            if (
                ((outputPoints.back().position - target_position.position).norm() < MPC_OBJECTIVE_TOLERANCE) |
                ((outputPoints.back().position - tp.position).norm() < MPC_CONSECUTIVE_POINT_TOLERANCE)
                )
            {
                break;
            }
        }
        
        outputPoints.push_back(tp);

    }

    TrajectoryConfig config;
    double duration = (outputPoints.size()-1) * DELTA_T;
    cout << "Duration of SampledTrajectory generated: " << duration << endl;
    std::shared_ptr<SampledTrajectory > st(new SampledTrajectory(config, outputPoints, duration));

    return st;
}


