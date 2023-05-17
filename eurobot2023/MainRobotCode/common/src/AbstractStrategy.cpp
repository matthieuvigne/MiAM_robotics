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

//--------------------------------------------------------------------------------------------------

bool AbstractStrategy::go_to_straight_line(RobotPosition position, double factor, bool backward)
{
  miam::trajectory::TrajectoryConfig conf = motionController->robotParams_.getTrajConf();
  conf.maxWheelVelocity *= factor;
  conf.maxWheelAcceleration *= factor;
  RobotPosition currentPosition = motionController->getCurrentPosition();
  TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
      conf,
      currentPosition, // start
      position, // end
      0.0, // no velocity at end point
      backward // or forward
  );

  motionController->setTrajectoryToFollow(traj);

  return motionController->waitForTrajectoryFinished();
}

//--------------------------------------------------------------------------------------------------

bool AbstractStrategy::turn_around_point(double angle_rad, double factor)
{
  miam::trajectory::TrajectoryConfig conf = motionController->robotParams_.getTrajConf();
  conf.maxWheelVelocity *= factor;
  conf.maxWheelAcceleration *= factor;
  //~ if(robot->isPlayingRightSide()) angle_rad = - angle_rad;
  RobotPosition currentPosition = motionController->getCurrentPosition();
  TrajectoryVector traj;
  traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(conf, currentPosition,
    currentPosition.theta + angle_rad)));
  motionController->setTrajectoryToFollow(traj);
  return motionController->waitForTrajectoryFinished();
}

//--------------------------------------------------------------------------------------------------

bool AbstractStrategy::go_to_rounded_corner(std::vector<RobotPosition> positions, bool backwards)
{
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
        robot->getParameters().getTrajConf(),
        positions,
        200,
        0.2,
        backwards
    );

    motionController->setTrajectoryToFollow(traj);

    return motionController->waitForTrajectoryFinished();
}

//--------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------

void AbstractStrategy::testSquare(bool clockwise, double const& squareDimension)
{
    RobotPosition position;
    position.x = 0;
    position.y = 0;
    position.theta = 0;
    motionController->resetPosition(position, true, true, true);
    double sgn = clockwise ? 1. : -1.;

    RobotPosition endPosition = position;
    endPosition.x += squareDimension;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

    position = motionController->getCurrentPosition();
    endPosition = position;
    endPosition.y -= sgn*squareDimension;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

    position = motionController->getCurrentPosition();
    endPosition = position;
    endPosition.x -= squareDimension;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();


    position = motionController->getCurrentPosition();
    endPosition = position;
    endPosition.y += sgn*squareDimension;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

    position = motionController->getCurrentPosition();
    endPosition = position;
    endPosition.x += squareDimension;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        position, endPosition);
    traj.pop_back();
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();


    bool moveSuccess = motionController->waitForTrajectoryFinished();
}

//--------------------------------------------------------------------------------------------------
