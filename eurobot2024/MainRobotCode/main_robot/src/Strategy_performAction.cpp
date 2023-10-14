#include "main_robot/Strategy.h"
#include "main_robot/MainRobotAction.h"

using namespace miam::trajectory;
using miam::RobotPosition;

namespace main_robot
{

bool Strategy::performMainRobotAction(Action* action)
{
    TrajectoryVector traj;

    // go to start taking obstacles into account
    for (auto obstacle : action->obstacles_on_the_road)
    {
        robot->getMotionController()->addPersistentObstacle(obstacle);
    }

    traj = robot->getMotionController()->computeMPCTrajectory(action->start_position, robot->getMotionController()->getDetectedObstacles(), true);

    // remove obstacles on the road
    for (auto obstacle : action->obstacles_on_the_road)
    {
        robot->getMotionController()->popBackPersistentObstacles();
    }

    if (traj.empty())
    {
        robot->logger_ << "[Strategy (secondary_robot)] " << "Motion planning to action failed!" << std::endl;
        return false;
    }
    robot->getMotionController()->setTrajectoryToFollow(traj);
    // if (action->needRailTop())
    // {
    //     moveRail(rail::ANTICIPATING_TOP);
    // }

    // go to start point and perform action
    if (!robot->getMotionController()->waitForTrajectoryFinished())
    {
        // action was not successful
        return false;
    }
    if (!action->performAction())
    {
        // action was not successful
        return false;
    }

    // action was successful
    // update score
    robot->updateScore(action->getScore());

    // add obstacle in the end
    for (auto obstacle : action->obstacles_in_the_end)
    {
        robot->getMotionController()->addPersistentObstacle(obstacle);
    }

    return true;

}

}