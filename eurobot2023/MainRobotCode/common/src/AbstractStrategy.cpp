#include "common/AbstractStrategy.h"
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>


using namespace miam;
using namespace miam::trajectory;

bool AbstractStrategy::go_to_straight_line(RobotPosition targetPosition, bool backward) 
{
    RobotPosition currentPosition = motionController->getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
        robot->getParameters().getTrajConf(),
        currentPosition, // start
        targetPosition, // end
        0.0, // no velocity at end point
        backward // or forward
    );

    motionController->setTrajectoryToFollow(traj);

    return motionController->waitForTrajectoryFinished();
}

bool AbstractStrategy::go_to_rounded_corner(std::vector<RobotPosition> targetPositions, bool backwards) 
{
    RobotPosition currentPosition = motionController->getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
        robot->getParameters().getTrajConf(),
        targetPositions, // end
        200,
        0.6,
        backwards
    );

    motionController->setTrajectoryToFollow(traj);

    return motionController->waitForTrajectoryFinished();
}

bool AbstractStrategy::go_forward(double distance) 
{
    RobotPosition currentPosition = motionController->getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLine(
        robot->getParameters().getTrajConf(),
        currentPosition, // start
        distance
    );

    motionController->setTrajectoryToFollow(traj);

    return motionController->waitForTrajectoryFinished();
}