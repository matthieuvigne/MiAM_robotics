#include "common/MotionPlanner.h"
#include <common/MotionParameters.h>

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/MathUtils.h>
#include <miam_utils/trajectory/PathPlanner.h>


#include <iostream>
#include <cmath>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include "tracy/Tracy.hpp"

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

// The parameters used to generate the MPC code
#define MPC_DELTA_T 0.1// 100 ms
#define MPC_N_TIME_INTERVALS 20 // 20 discrete time intervals

#define MPC_MU_TRAJ 100 // weight of the trajectory (x, y) in the optimization algorithm
#define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
#define MPC_MU_VLIN 0.01 // weight of the trajectory (v) in the optimization algorithm
#define MPC_MU_VANG 0.01 // weight of the trajectory (w) in the optimization algorithm

#define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

#define DELTA_T    MPC_DELTA_T
#define HORIZON_T   DELTA_T * N

#define MPC_OBJECTIVE_TOLERANCE 10 // mm
#define MPC_CONSECUTIVE_POINT_TOLERANCE 1 // mm
#define MPC_CONSECUTIVE_ANGLE_TOLERANCE 0.05 // rad

#define MPC_VELOCITY_OVERHEAD_PCT 0.9
#define MPC_ACCELERATION_OVERHEAD_PCT 0.9

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace miam;
using namespace std;

#ifdef MOTIONCONTROLLER_UNITTEST
std::vector<RobotPosition> UNITTEST_ASTAR_POS;
TrajectoryVector UNITTEST_POINTTURN_TRAJ;
TrajectoryVector UNITTEST_ROUNDED_TRAJ;
double UNITTEST_planningComputeDuration;
double UNITTEST_mpcComputeDuration;
double UNITTEST_printDuration;
#endif

// ACADO should be inited only once
bool is_acado_inited = false;
std::mutex acado_mutex;


MotionPlanner::MotionPlanner(RobotParameters const& robotParameters, Logger* logger) :
    robotParams_(robotParameters),
    logger_(logger)
{
}

TrajectoryVector MotionPlanner::planMotion(
            Map & map,
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition,
            tf const& flags,
            bool useTrajectoryRoundedCorners)
{
    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

    // To enforce end angle, try to reach a position offsetted from the true target.
    std::vector<RobotPosition > planned_path;
{
    ZoneScopedN("planMotion::PathPlanner");

    if (false)
    {
        double const OFFSET_DISTANCE = 50;
        RobotPosition offsetPosition = targetPosition - RobotPosition(OFFSET_DISTANCE, 0, 0).rotate(targetPosition.theta);
        if ((offsetPosition - currentPosition).norm() < (targetPosition - currentPosition).norm())
        {
            planned_path = miam::trajectory::planPath(map, currentPosition, offsetPosition, logger_);
            if (planned_path.size() > 0)
            {
                planned_path.push_back(targetPosition);
            }
            else
                *logger_ << "[PathPlanner] Planning with angle offset failed, try to replan to target" << std::endl;
        }
    }
    // ensureEndAngle = true;
    // If planning failed, or if no offset was asked, plan toward true target.
    if (planned_path.size() == 0)
        planned_path = miam::trajectory::planPath(map, currentPosition, targetPosition, logger_);

    std::chrono::high_resolution_clock::time_point astarTime = std::chrono::high_resolution_clock::now();
    *logger_ << "A* solved in: " << std::chrono::duration_cast<std::chrono::duration<double>>(astarTime - startTime).count() << std::endl;
}

#ifdef MOTIONCONTROLLER_UNITTEST
    std::chrono::high_resolution_clock::time_point printStart = std::chrono::high_resolution_clock::now();
#endif

{
    ZoneScopedN("planMotion::printMap");
    map.print(logger_, planned_path, currentPosition, targetPosition);
}
#ifdef MOTIONCONTROLLER_UNITTEST
    std::chrono::high_resolution_clock::time_point printEnd = std::chrono::high_resolution_clock::now();
    UNITTEST_printDuration = std::chrono::duration_cast<std::chrono::duration<double>>(printEnd - printStart).count();
    UNITTEST_ASTAR_POS = planned_path;
    // UNITTEST_planningComputeDuration = std::chrono::duration_cast<std::chrono::duration<double>>(astarTime - startTime).count();

    UNITTEST_POINTTURN_TRAJ = TrajectoryVector();
    if (planned_path.size() > 0)
    {
        RobotPosition pastPosition = planned_path.at(0);
        for (unsigned int i = 1; i < planned_path.size(); i++)
        {
            TrajectoryVector inc = miam::trajectory::computeTrajectoryStraightLineToPoint(
                robotParams_.getTrajConf(), pastPosition, planned_path.at(i));
            UNITTEST_POINTTURN_TRAJ = UNITTEST_POINTTURN_TRAJ + inc;
            pastPosition = inc.getEndPoint().position;
        }
        for (unsigned int i = 0; i < planned_path.size(); i++)
            *logger_ << "Planned point: " << planned_path.at(i) << std::endl;
    }

    UNITTEST_ROUNDED_TRAJ = computeTrajectoryRoundedCorner(
            robotParams_.getTrajConf(),
            planned_path,
            200,
            0.5,
            flags
        );
#endif

    // If path planning failed, return empty traj
    if (planned_path.size() == 0)
    {
        *logger_ << "[MotionPlanner] " << "Path planning did not find any path" << std::endl;
        return TrajectoryVector();
    }

#ifdef MOTIONCONTROLLER_UNITTEST
    // Store corresponding path
    for (unsigned int i = 0; i < planned_path.size(); i++)
    {
        logger_->log("PlannedPath.x", 0.15 * i, planned_path.at(i).x);
        logger_->log("PlannedPath.y", 0.15 * i, planned_path.at(i).y);
    }
#endif


    // compute the trajectory from the waypoints
    TrajectoryVector solvedTrajectory;

{
    ZoneScopedN("planMotion::solveTrajectoryFromWaypoints");
    if (useTrajectoryRoundedCorners)
    {
        *logger_ << "[MotionPlanner] " << "Smoothing path using trajectory rounded corners" << std::endl;

        solvedTrajectory = computeTrajectoryRoundedCorner(
            robotParams_.getTrajConf(),
            planned_path,
            200,
            0.5,
            flags
        );
    }
    else
    {
        *logger_ << "[MotionPlanner] " << "Smoothing path using MPC" << std::endl;

        planned_path.back().theta = targetPosition.theta;   // this makes the MPC try to ensure the end angle
                                                            // but end angle is often not reachable so the angle
                                                            // is then ensured with a point turn afterwards
        solvedTrajectory = solveTrajectoryFromWaypoints(planned_path, flags);
    }
}
#ifdef MOTIONCONTROLLER_UNITTEST
    // Store corresponding path
    double t = 0;
    while (t <= solvedTrajectory.getDuration())
    {
        TrajectoryPoint p = solvedTrajectory.getCurrentPoint(t);
        logger_->log("MPCOutput.x", t, p.position.x);
        logger_->log("MPCOutput.y", t, p.position.y);
        logger_->log("MPCOutput.theta", t, p.position.theta);
        logger_->log("MPCOutput.v", t, p.linearVelocity);
        logger_->log("MPCOutput.w", t, p.angularVelocity);

        p = UNITTEST_ROUNDED_TRAJ.getCurrentPoint(t);
        logger_->log("RoundedCornersTraj.x", t, p.position.x);
        logger_->log("RoundedCornersTraj.y", t, p.position.y);
        logger_->log("RoundedCornersTraj.theta", t, p.position.theta);
        logger_->log("RoundedCornersTraj.v", t, p.linearVelocity);
        logger_->log("RoundedCornersTraj.w", t, p.angularVelocity);

        t += 0.01;
    }
#endif


    TrajectoryVector finalTrajectory;

    if (solvedTrajectory.getDuration() > 0)
    {
        *logger_ << "[MotionPlanner] " << "Solved trajectory: " << std::endl;
        *logger_ << "[MotionPlanner] " << "Duration: " << solvedTrajectory.getDuration() << std::endl;
        *logger_ << "[MotionPlanner] " << "Start: " << solvedTrajectory.getCurrentPoint(0.0) << std::endl;
        *logger_ << "[MotionPlanner] " << "End: " << solvedTrajectory.getEndPoint() << std::endl;


        // at start, perform a point turn to get the right angle
        std::shared_ptr<PointTurn > pt_sub_start(
            new PointTurn(robotParams_.getTrajConf(),
            currentPosition, solvedTrajectory.getCurrentPoint(0.0).position.theta));
        finalTrajectory.push_back(pt_sub_start);

        // insert solved trajectory
        finalTrajectory = finalTrajectory + solvedTrajectory;

#ifdef MOTIONCONTROLLER_UNITTEST
    UNITTEST_ROUNDED_TRAJ.insert(UNITTEST_ROUNDED_TRAJ.begin(), pt_sub_start);
#endif
        if (!(flags & tf::IGNORE_END_ANGLE))
        {
            // at end, perform a point turn to get the right angle
            std::shared_ptr<PointTurn> pt_sub_end(
                new PointTurn(robotParams_.getTrajConf(),
                finalTrajectory.getEndPoint().position,
                targetPosition.theta)
            );
            finalTrajectory.push_back(pt_sub_end);
#ifdef MOTIONCONTROLLER_UNITTEST
    UNITTEST_ROUNDED_TRAJ.push_back(pt_sub_end);
#endif
        }
    }
    else
    {
        *logger_ << "[MotionPlanner] " << "Solver failed" << std::endl;
    }

    return finalTrajectory;
}


TrajectoryVector MotionPlanner::solveTrajectoryFromWaypoints(
    std::vector<RobotPosition> waypoints,
    tf const& flags
)
{
    trajectory::TrajectoryVector traj;

    // parameterize solver
    miam::trajectory::TrajectoryConfig cplan = robotParams_.getTrajConf();
    cplan.maxWheelVelocity *= MPC_VELOCITY_OVERHEAD_PCT; // give some overhead to the controller
    cplan.maxWheelAcceleration *= MPC_ACCELERATION_OVERHEAD_PCT; // give some overhead to the controller

    // create trajectory interpolating waypoints
    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
    traj = computeTrajectoryBasicPath(cplan, waypoints, 0, flags);

#ifdef MOTIONCONTROLLER_UNITTEST
    // Store corresponding path
    double t = 0;
    while (t <= traj.getDuration())
    {
        TrajectoryPoint p = traj.getCurrentPoint(t);
        logger_->log("MPCReference.x", t, p.position.x);
        logger_->log("MPCReference.y", t, p.position.y);
        logger_->log("MPCReference.theta", t, p.position.theta);
        logger_->log("MPCReference.v", t, p.linearVelocity);
        logger_->log("MPCReference.w", t, p.angularVelocity);
        t += 0.01;
    }
#endif

    // start of the entire trajectory
    TrajectoryPoint start_position = traj.getCurrentPoint(0.0);
    TrajectoryPoint target_position = traj.getCurrentPoint(traj.getDuration());
    TrajectoryVector res;

    int nIterMax = ceil(traj.getDuration() / (HORIZON_T - 2 * DELTA_T)) + 2; // enable that many iterations
    int nIter = 0;

    // proceed by increments of HORIZON_T - 2 * DELTA_T (not taking the last point since the final
    // constraint might make the solver brutally go towards the final point
    while (nIter < nIterMax)
    {
        std::shared_ptr<SampledTrajectory > solvedTrajectory = solveMPCIteration(
            traj,
            start_position,
            target_position,
            nIter * (HORIZON_T - 2 * DELTA_T),
            flags
        );

        // this is the case in which the solved MPC iteration has some
        // stationary points ; the function truncates these points, which renders the duration less
        // than horizon_t
        // todo : maybe add some conditions about being close to the target?
        if (solvedTrajectory->getDuration() < HORIZON_T)
        {
            res.push_back(solvedTrajectory);
            break;
        }

        // remove last 2 points
        solvedTrajectory->removePoints(2);
        res.push_back(solvedTrajectory);

        start_position = solvedTrajectory->getCurrentPoint(HORIZON_T - 2 * DELTA_T);
        nIter++;
    }
    std::chrono::high_resolution_clock::time_point mpcTime = std::chrono::high_resolution_clock::now();
    *logger_ << "MPC solved in: " << std::chrono::duration_cast<std::chrono::duration<double>>(mpcTime - startTime).count() << std::endl;

#ifdef MOTIONCONTROLLER_UNITTEST
    UNITTEST_mpcComputeDuration = std::chrono::duration_cast<std::chrono::duration<double>>(mpcTime - startTime).count();
#endif
    return res;
}


TrajectoryVector MotionPlanner::computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed,
    tf const& flags)
{
    bool const& backward = flags & tf::BACKWARD;

    // Get total distance
    double distance = 0;
    for (unsigned int i = 0; i < p.size() - 1; i++)
    {
        distance += (p.at(i+1) - p.at(i)).norm();
    }

    // Build corresponding velocity trapezoid
    Trapezoid tp = Trapezoid(
        distance,
        initialSpeed,
        0.0,
        config.maxWheelVelocity,
        config.maxWheelAcceleration
    );

    // parameterize the trajectory using curvilinear abscissa
    double endPointCurvilinearAbscissa = 0.0;
    double startPointTrapezoid = 0.0;

    TrajectoryVector vector;
    for (long unsigned int i = 0; i < p.size() - 1; i++)
    {
        RobotPosition p1 = p.at(i);
        RobotPosition p2 = p.at(i+1);

        if (backward)
        {
            p1.theta += M_PI;
            p2.theta += M_PI;
        }

        endPointCurvilinearAbscissa += (p2 - p1).norm();

        double startSpeed = 0.0;
        if (i == 0) {
            startSpeed = initialSpeed;
        } else {
            // starting speed is the ending speed of the preceding trajectory
            // do not take the endpoint or else the speed is null
            startSpeed = vector.back()->getCurrentPoint(vector.back()->getDuration()).linearVelocity;
        }

        // get max velocity from tp
        double timeOfTrapezoid = startPointTrapezoid;
        while (timeOfTrapezoid < tp.getDuration() && tp.getState(timeOfTrapezoid).position < endPointCurvilinearAbscissa)
        {
            timeOfTrapezoid += 0.001;
        }
        double endSpeed = tp.getState(timeOfTrapezoid).velocity;

        std::shared_ptr<StraightLine> line(new StraightLine(
            config, p1, p2,
            std::abs(startSpeed),
            endSpeed,
            backward));

        // Go from point to point.
        vector.push_back(std::shared_ptr<Trajectory>(line));

        // Update variables
        startPointTrapezoid += line->getDuration();
    }

    return vector;
};

std::shared_ptr<SampledTrajectory > MotionPlanner::solveMPCIteration(
    TrajectoryVector reference_trajectory,
    TrajectoryPoint start_position,
    TrajectoryPoint target_position,
    double const& start_time,
    tf const& flags
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


        TrajectoryPoint tp;


        std::vector<double> posX, posY, posTheta, linVel, angVel;
        for (int indice = 0; indice < N+1; indice++)
        {
            real_t const t = indice * DELTA_T + start_time;
            tp = reference_trajectory.getCurrentPoint(t);

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

            posX.push_back(tp.position.x);
            posY.push_back(tp.position.y);
            posTheta.push_back(tp.position.theta);
            linVel.push_back(tp.linearVelocity);
            angVel.push_back(tp.angularVelocity);
        }

        // Do some padding on theta to better match the desired end angle.
        double const HALF_WINDOW_SIZE = 7;
        // if (tryEnsureEndAngle)
        // {
        //     for (int i = 0; i < HALF_WINDOW_SIZE; i++)
        //         posTheta.push_back(endAngle);
        // }
        // Filter
        posTheta = movingAverage(posTheta, HALF_WINDOW_SIZE);

        // Fill problem
        for (int indice = 0; indice < N+1; indice++)
        {
            /* Initialize the states. */
            acadoVariables.x[ indice * NX ] = posX.at(indice)/ 1000.0;
            acadoVariables.x[ indice * NX + 1] = posY.at(indice) / 1000.0;
            acadoVariables.x[ indice * NX + 2] = posTheta.at(indice);

            /* Initialize the controls. */
            acadoVariables.x[ indice * NX + 3] = linVel.at(indice) / 1000.0;
            acadoVariables.x[ indice * NX + 4] = angVel.at(indice);

            if (indice < N) {
                acadoVariables.y[ indice * NY ] = posX.at(indice) / 1000.0;
                acadoVariables.y[ indice * NY + 1] = posY.at(indice) / 1000.0;
                acadoVariables.y[ indice * NY + 2] = posTheta.at(indice);
                acadoVariables.y[ indice * NY + 3] = linVel.at(indice) / 1000.0;
                acadoVariables.y[ indice * NY + 4] = angVel.at(indice);
            }
        }

        acadoVariables.yN[0] = posX.back() / 1000.0;
        acadoVariables.yN[1] = posY.back() / 1000.0;
        acadoVariables.yN[2] = posTheta.back();
        acadoVariables.yN[3] = linVel.back() / 1000.0;
        acadoVariables.yN[4] = angVel.back();

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
                *logger_ << "[MotionPlanner] " << "solved: " << "t=" << t << " --- " <<
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
    std::shared_ptr<SampledTrajectory > solvedTrajectory(new SampledTrajectory(config, outputPoints, duration));



    acado_mutex.unlock();

    return solvedTrajectory;
}


