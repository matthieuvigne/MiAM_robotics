#include "common/MotionPlanner.h"
#include <common/MotionParameters.h>

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

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
#define MPC_CONSECUTIVE_ANGLE_TOLERANCE 0.05 // rad
#define MPC_VELOCITY_OVERHEAD_PCT 0.8

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
std::mutex acado_mutex;


TrajectoryConfig MotionPlanner::getMPCTrajectoryConfig() {
    TrajectoryConfig c;
    c.maxWheelVelocity = 1000;
    c.maxWheelAcceleration = 2000;
    c.robotWheelSpacing = 100.5;
    return c;
};

MotionPlanner::MotionPlanner(RobotParameters const& robotParameters) :
    robotParams_(robotParameters),
    pathPlanner_(PathPlannerConfig())
{

    // create pathPlanner with correct grid
    PathPlannerConfig config;
    config.astar_resolution_mm = 50;
    config.astar_grid_size_x = table_dimensions::table_size_x / 50;
    config.astar_grid_size_y = table_dimensions::table_size_y / 50;
    config.forbidden_border_size_mm = 1; // should be equal to robot radius
    pathPlanner_ = PathPlanner(config);
}

TrajectoryVector MotionPlanner::planMotion(
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition,
            bool ensureEndAngle,
            bool useTrajectoryRoundedCorners)
{
    // pathPlanner_.printMap();
    std::vector<RobotPosition > planned_path = pathPlanner_.planPath(currentPosition, targetPosition);
    pathPlanner_.printMap(planned_path, currentPosition, targetPosition);

    // If path planning failed, return empty traj
    if (planned_path.size() == 0)
    {
        textlog << "[MotionPlanner] " << "Path planning did not find any path" << std::endl;
        return TrajectoryVector();
    }

    TrajectoryVector res;
    textlog << "[MotionPlanner] " << "Start solving..." << std::endl;

    // compute the trajectory from the waypoints
    TrajectoryVector st;

    if (useTrajectoryRoundedCorners)
    {
        textlog << "[MotionPlanner] " << "Solving trajectory rounded corners " << std::endl;

        miam::trajectory::TrajectoryConfig cplan = getMPCTrajectoryConfig();
        cplan.maxWheelVelocity *= MPC_VELOCITY_OVERHEAD_PCT; // give 20% overhead to the controller

        st = computeTrajectoryRoundedCorner(
            cplan,
            planned_path,
            200
        );
    }
    else
    {
        planned_path.back().theta = targetPosition.theta;
        st = solveTrajectoryFromWaypoints(planned_path);
    }

    if (st.getDuration() > 0)
    {
        textlog << "[MotionPlanner] " << "Solved trajectory: " << std::endl;
        // for (double t = 0; t <=st.getDuration(); t += 0.1) {
        //     textlog << "[MotionPlanner] " << st.getCurrentPoint(t) << std::endl;
        // }
        textlog << "[MotionPlanner] " << "Duration: " << st.getDuration() << std::endl;
        textlog << "[MotionPlanner] " << "Start: " << st.getCurrentPoint(0.0) << std::endl;
        textlog << "[MotionPlanner] " << "End: " << st.getEndPoint() << std::endl;


        // at start, perform a point turn to get the right angle
        std::shared_ptr<PointTurn > pt_sub_start(
            new PointTurn(robotParams_.getTrajConf(),
            currentPosition, st.getCurrentPoint(0.0).position.theta));
        res.push_back(pt_sub_start);

        // insert solved trajectory
        res.insert( res.end(), st.begin(), st.end() );

        if (ensureEndAngle)
        {
            // at end, perform a point turn to get the right angle
            std::shared_ptr<PointTurn > pt_sub_end(
                new PointTurn(robotParams_.getTrajConf(),
                res.getEndPoint().position, targetPosition.theta)
            );
            res.push_back(pt_sub_end);
        }
    }
    else
    {
        textlog << "[MotionPlanner] " << "Solver failed" << std::endl;
    }

    return res;
}


TrajectoryVector MotionPlanner::solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints
)
{
    trajectory::TrajectoryVector traj;

    // parameterize solver
    miam::trajectory::TrajectoryConfig cplan = getMPCTrajectoryConfig();
    cplan.maxWheelVelocity *= MPC_VELOCITY_OVERHEAD_PCT; // give 20% overhead to the controller
    cplan.maxWheelAcceleration *= MPC_VELOCITY_OVERHEAD_PCT; // give 20% overhead to the controller

    // create trajectory interpolating waypoints
    traj = computeTrajectoryBasicPath(cplan, waypoints, 0);

    // start of the entire trajectory
    TrajectoryPoint start_position = traj.getCurrentPoint(0.0);
    TrajectoryPoint target_position = traj.getCurrentPoint(traj.getDuration());
    TrajectoryVector res;

    int nIterMax = ceil(traj.getDuration() / (HORIZON_T - 2 * DELTA_T)) + 2; // enable that many iterations
    int nIter = 0;

    cout << "Duration of the reference path: " << traj.getDuration() << endl;
    cout << "nIterMax: " << nIterMax << endl;

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

        if (st->getDuration() < HORIZON_T)
        {
            res.push_back(st);
            break;
        }

        // remove last 2 points
        st->removePoints(2);
        res.push_back(st);

        start_position = st->getCurrentPoint(HORIZON_T - 2 * DELTA_T);
        nIter++;
    }
    return res;
}


TrajectoryVector MotionPlanner::computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed)
{
    TrajectoryVector vector;

    // get total distance
    double distance = 0;
    for (long unsigned int i = 0; i < p.size() - 1; i++)
    {
        distance += (p.at(i+1) - p.at(i)).norm();
    }

    // make a trapezoid of velocity
    Trapezoid tp = Trapezoid(
        distance,
        initialSpeed,
        0.0,
        config.maxWheelVelocity,
        config.maxWheelAcceleration
    );

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

        // approximate the velocity curve to be a trapezoid
        // could refine that taking distance into account
        double maxSpeed = (
            tp.getState(tp.getDuration() * i / (p.size()-1)).velocity +
            tp.getState(tp.getDuration() * (i+1) / (p.size()-1)).velocity
        ) / 2.0;

        std::shared_ptr<StraightLine> line(new StraightLine(
            config, p1, p2,
            starting_speed,
            maxSpeed, false));

        // Go from point to point.
        vector.push_back(std::shared_ptr<Trajectory>(line));
    }

    if (VERBOSE)
    {
        for (double t=0; t <= vector.getDuration(); t+=0.2)
        {
            textlog << "[MotionPlanner] " << "t: " << t << " " << vector.getCurrentPoint(t) << std::endl;
        }
        textlog << "[MotionPlanner] " << "t: " << vector.getDuration() << " " << vector.getEndPoint() << std::endl;
    }

    return vector;
};

std::shared_ptr<SampledTrajectory > MotionPlanner::solveMPCIteration(
    TrajectoryVector reference_trajectory,
    TrajectoryPoint start_position,
    TrajectoryPoint target_position,
    double start_time
)
{
    acado_mutex.lock();

    if (VERBOSE)
    {
        cout << "----------------------------" << endl;
        cout << "Solving starting time: " <<  start_time << endl;
        cout << "Start position: " << start_position << endl;
        cout << "Target position: " << target_position << endl;
        cout << "N: " << N << endl;
    }


    std::vector<TrajectoryPoint> outputPoints;

    try {

        if (!is_acado_inited)
        {
            /* Initialize the solver. */
            cout << "Initializing solver" << endl;
            acado_initializeSolver();
            is_acado_inited = true;
        }

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
            //     textlog << "[MotionPlanner] " << "true: " << "t=" << t << " --- " << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
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

        for (int indice = 0; indice < N+1; indice++) {

            real_t t = indice * DELTA_T + start_time;

            tp.position.x      = acadoVariables.x[ indice * NX ] * 1000;
            tp.position.y      = acadoVariables.x[ indice * NX + 1] * 1000;
            tp.position.theta  = acadoVariables.x[ indice * NX + 2];
            tp.linearVelocity  = acadoVariables.x[ indice * NX + 3] * 1000;
            tp.angularVelocity = acadoVariables.x[ indice * NX + 4];

            if (VERBOSE)
            {
                textlog << "[MotionPlanner] " << "solved: " << "t=" << t << " --- " <<
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
                    // we are very close to the target in norm
                    ((outputPoints.back().position - target_position.position).norm() < MPC_OBJECTIVE_TOLERANCE) ||
                    // time is greater than reference duration and
                    // two consecutive points in the trajectory are too similar in norm and in angle
                    (
                        (outputPoints.size()-1) * DELTA_T > reference_trajectory.getDuration() &&
                        (outputPoints.back().position - tp.position).norm() < MPC_CONSECUTIVE_POINT_TOLERANCE &&
                        std::abs(outputPoints.back().position.theta - tp.position.theta) < MPC_CONSECUTIVE_ANGLE_TOLERANCE
                    )
                    )
                {
                    break;
                }
            }

            outputPoints.push_back(tp);

        }
    } catch( const exception & e ) {
        cerr << "ACADO failed!" << endl;
        cerr << e.what() << endl;
    }

    TrajectoryConfig config;
    double duration = std::max(0.0, (outputPoints.size()-1) * DELTA_T);
    if (VERBOSE)
        cout << "Duration of SampledTrajectory generated: " << duration << endl;
    std::shared_ptr<SampledTrajectory > st(new SampledTrajectory(config, outputPoints, duration));



    acado_mutex.unlock();

    return st;
}


