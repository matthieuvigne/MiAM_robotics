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
    RobotPosition startPosition = motionController->getCurrentPosition();
    double const startAngle = startPosition.theta;
    double sgn = clockwise ? 1. : -1.;

    RobotPosition endPosition = startPosition + RobotPosition(squareDimension, 0.0).rotate(startAngle);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        startPosition, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    startPosition = traj.getEndPoint().position;
    endPosition = endPosition + sgn * RobotPosition(0.0, -squareDimension).rotate(startAngle);
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        startPosition, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    startPosition = traj.getEndPoint().position;
    endPosition = endPosition + RobotPosition(-squareDimension, 0.0).rotate(startAngle);
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        startPosition, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    startPosition = traj.getEndPoint().position;
    endPosition = endPosition + sgn * RobotPosition(0.0, squareDimension).rotate(startAngle);
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        startPosition, endPosition, 0.0, tf::IGNORE_END_ANGLE);

    startPosition = traj.getEndPoint().position;
    endPosition = endPosition + RobotPosition(squareDimension, 0.0).rotate(startAngle);
    traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        startPosition, endPosition, 0.0, tf::IGNORE_END_ANGLE);
    traj.pop_back();
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

}

//--------------------------------------------------------------------------------------------------
