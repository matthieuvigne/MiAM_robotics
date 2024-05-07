#include "common/AbstractStrategy.h"
#include <unistd.h>
#include <math.h>
#include <thread>
#include <algorithm>

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>


using namespace miam;
using namespace miam::trajectory;



//--------------------------------------------------------------------------------------------------

void AbstractStrategy::testSquare(bool clockwise, double const& squareDimension)
{
    RobotPosition position;
    position.x = 500;
    position.y = 500;
    position.theta = 0;
    motionController->resetPosition(position, true, true, true);
    double sgn = clockwise ? 1. : -1.;

    RobotPosition endPosition = position;
    endPosition.x += squareDimension;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    position = traj.getEndPoint().position;
    endPosition.y -= sgn*squareDimension;
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    position = traj.getEndPoint().position;
    position.theta = -sgn * M_PI_2;
    endPosition.x -= squareDimension;
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    position = traj.getEndPoint().position;
    position.theta = -sgn * M_PI;
    endPosition.y += sgn*squareDimension;
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    position = traj.getEndPoint().position;
    position.theta = -sgn * 3 * M_PI_2;
    endPosition.x += squareDimension;
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition, 0.0, tf::IGNORE_END_ANGLE);
    traj.pop_back();
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

}

//--------------------------------------------------------------------------------------------------
