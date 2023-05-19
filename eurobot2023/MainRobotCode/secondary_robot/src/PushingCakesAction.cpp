#include <secondary_robot/PushingCakesAction.h>

bool PushingCakesAction::performAction(AbstractStrategy* strategy)
{
    RobotInterface* robot = strategy->robot;

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


    // useless path planner already does that
    // currentPosition = robot->getMotionController()->getCurrentPosition();
    // traj = miam::trajectory::computeTrajectoryStraightLine(
    //     robot->getParameters().getTrajConf(),
    //     currentPosition, // start
    //     -200 // back a lot
    // );

    // robot->getMotionController()->setTrajectoryToFollow(traj);
    // robot->getMotionController()->waitForTrajectoryFinished();

    // perform action
    // action was successful
    return(pushingMoveSucceeded);
}