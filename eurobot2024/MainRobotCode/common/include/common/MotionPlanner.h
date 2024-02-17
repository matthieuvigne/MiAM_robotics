#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <miam_utils/trajectory/PathPlanner.h>
#include <miam_utils/TextLogger.h>
#include "common/RobotParameters.h"

#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H

using namespace miam;
using namespace miam::trajectory;
class MotionPlanner{

    public:

        MotionPlanner(RobotParameters const& robotParameters);

        TrajectoryVector planMotion(
            RobotPosition const& currentPosition,
            RobotPosition const& targetPosition,
            bool ensureEndAngle,
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

        static TrajectoryVector computeTrajectoryBasicPath(
            TrajectoryConfig const& config,
            std::vector<RobotPosition> p,
            double initialSpeed);

        static TrajectoryVector solveTrajectoryFromWaypoints(
            std::vector<RobotPosition> waypoints,
            bool const& tryEnsureEndAngle,
            double const& endAngle
        );

        static TrajectoryConfig getMPCTrajectoryConfig();

        static std::shared_ptr<SampledTrajectory > solveMPCIteration(
            TrajectoryVector reference_trajectory,
            TrajectoryPoint start_position,
            TrajectoryPoint target_position,
            double const& start_time,
            bool const& tryEnsureEndAngle,
            double const& endAngle
        );

};




#endif