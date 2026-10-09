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

#include "MPCParameters.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define N           ACADO_N   /* Number of intervals in the horizon. */
#define HORIZON_T   MPC_DELTA_T * N

#define VERBOSE     false         /* Show iterations: 1, silent: 0.  */


/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


// Validation constants
#define MPC_OBJECTIVE_TOLERANCE 25 // mm
#define MPC_CONSECUTIVE_POINT_TOLERANCE 1 // mm
#define MPC_CONSECUTIVE_ANGLE_TOLERANCE 0.05 // rad

#define MPC_VELOCITY_OVERHEAD_PCT 0.9
#define MPC_ACCELERATION_OVERHEAD_PCT 0.9


using namespace miam;

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


MotionPlanner::MotionPlanner(Logger* logger) :
    logger_(logger)
{
}

TrajectoryVector MotionPlanner::planMotion(
            TrajectoryConfig const& config,
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
            planned_path = miam::trajectory::planPath(map, currentPosition, offsetPosition);
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
        planned_path = miam::trajectory::planPath(map, currentPosition, targetPosition);

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
                config, pastPosition, planned_path.at(i));
            UNITTEST_POINTTURN_TRAJ = UNITTEST_POINTTURN_TRAJ + inc;
            pastPosition = inc.getEndPoint().position;
        }
        for (unsigned int i = 0; i < planned_path.size(); i++)
            *logger_ << "Planned point: " << planned_path.at(i) << std::endl;
    }

    UNITTEST_ROUNDED_TRAJ = computeTrajectoryRoundedCorner(
            config,
            planned_path,
            200,
            0.5,
            flags
        );
#endif

    // If path planning failed, return empty traj
    if (planned_path.size() == 0)
    {
        *logger_ << "[MotionPlanner] Path planning did not find any path" << std::endl;
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
            config,
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
        solvedTrajectory = solveTrajectoryFromWaypoints(config, planned_path, flags);
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
            new PointTurn(config,
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
                new PointTurn(config,
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
    miam::trajectory::TrajectoryConfig const& config,
    std::vector<RobotPosition> waypoints,
    tf const& flags
)
{
    trajectory::TrajectoryVector traj;

    // parameterize solver
    miam::trajectory::TrajectoryConfig cplan = config;
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

#define OVERLAP 6
    int nIterMax = ceil(traj.getDuration() / (HORIZON_T - OVERLAP * MPC_DELTA_T)) + 2; // enable that many iterations
    int nIter = 0;

    // proceed by increments of HORIZON_T - 2 * MPC_DELTA_T (not taking the last point since the final
    // constraint might make the solver brutally go towards the final point
    while (nIter < nIterMax)
    {
        std::shared_ptr<SampledTrajectory > solvedTrajectory = solveMPCIteration(
            traj,
            start_position,
            target_position,
            nIter * (HORIZON_T - OVERLAP * MPC_DELTA_T),
            flags
        );

        if (solvedTrajectory->getDuration() < std::numeric_limits<double>::epsilon())
        {
            // MPC failed, abort
            return TrajectoryVector();
        }
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
        solvedTrajectory->removePoints(OVERLAP);
        res.push_back(solvedTrajectory);

        start_position = solvedTrajectory->getEndPoint();
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

std::shared_ptr<SampledTrajectory> MotionPlanner::solveMPCIteration(
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
        *logger_ << "----------------------------" << std::endl;
        *logger_ << "Solving starting time: " <<  start_time << std::endl;
        *logger_ << "Start position: " << start_position << std::endl;
        *logger_ << "Target position: " << target_position << std::endl;
        *logger_ << "N: " << N << std::endl;
    }


    std::vector<TrajectoryPoint> outputPoints;

    try {

        if (!is_acado_inited)
        {
            /* Initialize the solver. */
            *logger_ << "Initializing solver" << std::endl;
            acado_initializeSolver();
            is_acado_inited = true;
        }


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
            real_t const t = indice * MPC_DELTA_T + start_time;
            tp = reference_trajectory.getCurrentPoint(t);

            // if the gap is > M_PI, then subtract 2 pi
            if ( tp.position.theta - oldtp.position.theta > M_PI)
            {
                tp.position.theta =  tp.position.theta - 2 * M_PI;
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
        // double const HALF_WINDOW_SIZE = 7;
        // if (tryEnsureEndAngle)
        // {
        //     for (int i = 0; i < HALF_WINDOW_SIZE; i++)
        //         posTheta.push_back(endAngle);
        // }
        // Filter
        // posTheta = movingAverage(posTheta, HALF_WINDOW_SIZE);

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
        int rc = acado_feedbackStep( );
        if (rc != 0)
        {
            *logger_ << "[MotionPlanner] QPOases failed to solve problem: " << rc << std::endl;
            // acado_mutex.unlock();
            // return std::make_shared<SampledTrajectory>(TrajectoryConfig(), std::vector<TrajectoryPoint>(), 0.0);
        }

        /* Apply the new control immediately to the process, first NU components. */

        if( VERBOSE )
            *logger_ << "\t KKT Tolerance = " <<  acado_getKKT() << std::endl;

        /* Read the elapsed time. */
        real_t te = acado_toc( &t );

        if( VERBOSE )
            *logger_ << "Average time of one real-time iteration:" << 1e6 * te << "microseconds" << std::endl;

        for (int indice = 0; indice < N+1; indice++) {

            real_t t = indice * MPC_DELTA_T + start_time;

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
                        (outputPoints.size()-1) * MPC_DELTA_T > reference_trajectory.getDuration() &&
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
    } catch( const std::exception & e ) {
        *logger_ << "ACADO failed!" << std::endl;
        *logger_ << e.what() << std::endl;
    }

    TrajectoryPoint tp = target_position;
    if ( tp.position.theta - outputPoints.back().position.theta > M_PI)
    {
        tp.position.theta =  tp.position.theta - 2 * M_PI;
    }
    // if the gap is < -M_PI, then add 2 pi
    if ( tp.position.theta - outputPoints.back().position.theta < -M_PI)
    {
        tp.position.theta = tp.position.theta + 2 * M_PI;
    }
    outputPoints.push_back(tp);
    TrajectoryConfig config;
    double duration = std::max(0.0, (outputPoints.size()-1) * MPC_DELTA_T);
    if (VERBOSE)
        *logger_ << "Duration of SampledTrajectory generated: " << duration << std::endl;
    std::shared_ptr<SampledTrajectory > solvedTrajectory(new SampledTrajectory(config, outputPoints, duration));

    acado_mutex.unlock();

    return solvedTrajectory;
}


