#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/PathPlanner.h>
#include <miam_utils/Logger.h>
#include "common/RobotParameters.h"

#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H

using namespace miam;
using namespace miam::trajectory;

#ifdef MOTIONCONTROLLER_UNITTEST
extern std::vector<RobotPosition> UNITTEST_ASTAR_POS;
extern TrajectoryVector UNITTEST_POINTTURN_TRAJ;
extern TrajectoryVector UNITTEST_ROUNDED_TRAJ;
#endif

using tf = miam::trajectory::flags;

class MotionPlanner{

    public:

        MotionPlanner(RobotParameters const& robotParameters, Logger *logger);

        TrajectoryVector planMotion(
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition,
            tf const& flags,
            bool useTrajectoryRoundedCorners = false);


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

        RobotParameters robotParams_;
        PathPlanner pathPlanner_;

        /// @brief Compute interpolation trajectory between a list of points, ignoring rotations.
        TrajectoryVector computeTrajectoryBasicPath(
            TrajectoryConfig const& config,
            std::vector<RobotPosition> p,
            double initialSpeed,
            tf const& flags);

        TrajectoryVector solveTrajectoryFromWaypoints(
            std::vector<RobotPosition> waypoints,
            tf const& flags
        );

        std::shared_ptr<SampledTrajectory > solveMPCIteration(
            TrajectoryVector reference_trajectory,
            TrajectoryPoint start_position,
            TrajectoryPoint target_position,
            double const& start_time,
            tf const& flags
        );

        Logger* logger_;
};




#endif