#include <secondary_robot/PushingCakesAction.h>

bool PushingCakesAction::performAction(RobotInterface* robot)
{
    RobotPosition currentPosition;
    TrajectoryVector traj;
    miam::trajectory::TrajectoryConfig conf = robot->getMotionController()->robotParams_.getTrajConf();

    currentPosition = robot->getMotionController()->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
        conf,
        currentPosition, // start
        end_position, // end
        0.0, // no velocity at end point
        false // or forward
    );

    robot->getMotionController()->setTrajectoryToFollow(traj);
    bool pushingMoveSucceeded = robot->getMotionController()->waitForTrajectoryFinished();

    currentPosition = robot->getMotionController()->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(
        robot->getParameters().getTrajConf(),
        currentPosition, // start
        -100 // back a little
    );

    robot->getMotionController()->setTrajectoryToFollow(traj);
    robot->getMotionController()->waitForTrajectoryFinished();

    // perform action
    // action was successful
    return(pushingMoveSucceeded);
}