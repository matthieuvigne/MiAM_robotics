#include "main_robot/Strategy.h"
#include "main_robot/MainRobotAction.h"

// #define TESTING_MPC


using namespace miam::trajectory;
using miam::RobotPosition;

namespace main_robot
{

void Strategy::match_impl()
{
    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    MotionPlanner* motionPlanner = motionController->getMotionPlanner();
    std::vector<Action> actionVector;
    RobotParameters const robotParameters = robot->getParameters();
    RobotPosition tmp_position;
    RobotPosition current_position = motionController->getCurrentPosition();

    // go_forward(1000);

#ifdef TESTING_MPC
    endPosition.x = 2287;
    endPosition.y = 1189;
    endPosition.theta = M_PI_2;
    traj = robot->getMotionController()->computeMPCTrajectory(endPosition, robot->getMotionController()->getDetectedObstacles(), true);
    if (!traj.empty())
    {
        robot->getMotionController()->setTrajectoryToFollow(traj);
        robot->getMotionController()->waitForTrajectoryFinished();
    }

    return;
#endif

    std::vector<std::shared_ptr<Action > > actions;

    std::vector<RobotPosition > plants_positions;
    endPosition.x = 1500;
    endPosition.y = 1550;
    plants_positions.push_back(endPosition);
    endPosition.x = 1000;
    endPosition.y = 1300;
    plants_positions.push_back(endPosition);
    endPosition.x = 1000;
    endPosition.y = 700;
    plants_positions.push_back(endPosition);

    for (auto& plant_position : plants_positions)
    {
        std::shared_ptr<Action > newAction(new GetPlantsAction(this));
        newAction->start_position = plant_position;
        actions.push_back(newAction);
    }

    int number_of_unsuccessful_iters = 0;

    while (actions.size() > 0)
    {
        // which action is :
        // * so that no robot is close
        // * and the closer to the current position

        int action_index = -1;
        double minDistanceToStartPoint = 10000;
        RobotPosition currentPosition = robot->getMotionController()->getCurrentPosition();
        for (int i = 0; i < actions.size(); i++)
        {

            Action* action = actions.at(i).get();

            if (!action->activated)
            {
                robot->logger_ << "[Strategy (main_robot)] " << "action " << i << " was deactivated" << std::endl;
                continue;
            }

            double minDistanceFromObstacle = 10000;
            double minDistanceFromObstacleEnd = 10000;
            double distanceToStartPoint = (action->start_position - currentPosition).norm();

            for (auto obstacle : robot->getMotionController()->getDetectedObstacles())
            {

                double tmpMin = (std::get<0>(obstacle) - action->start_position).norm() - std::get<1>(obstacle);
                double tmpMinEnd = (std::get<0>(obstacle) - action->end_position).norm() - std::get<1>(obstacle);

                // distance to center of obstacle minus size of the obstacle
                minDistanceFromObstacle = std::min(
                    minDistanceFromObstacle,
                    tmpMin);
                // distance to center of obstacle minus size of the obstacle
                minDistanceFromObstacleEnd = std::min(
                    minDistanceFromObstacleEnd,
                    tmpMinEnd);
            }

            // 150 = radius of the robot
            if (minDistanceFromObstacle > 20 & minDistanceFromObstacleEnd > 20 &
                distanceToStartPoint < minDistanceToStartPoint)
            {
                action_index = i;
                minDistanceToStartPoint = distanceToStartPoint;
            }


            robot->logger_ << "[Strategy (main_robot)] " << "action " << i << " start point " << action->start_position << std::endl;
            robot->logger_ << "[Strategy (main_robot)] " << "   minDistanceFromObstacle " << minDistanceFromObstacle << " minDistanceFromObstacleEnd " << minDistanceFromObstacleEnd << " distanceToStartPoint " << distanceToStartPoint << std::endl;

        }

        robot->logger_ << "[Strategy (main_robot)] " << "Chosen action: " << action_index << std::endl;
        if (action_index < 0)
        {
            number_of_unsuccessful_iters++;
            robot->logger_ << "[Strategy (main_robot)] " << "Reactivating all actions" << std::endl;
            for (int i = 0; i < actions.size() ; i++)
            {
                actions.at(i)->activated = true;
            }
            // robot->wait(1);
        }
        else
        {
            number_of_unsuccessful_iters = 0;

            robot->logger_ << "[Strategy (main_robot)] " << "####### Performing pushing action: " << action_index << " (" << actions.size() << " remaining)" << std::endl;

            Action* action = actions.at(action_index).get();

            if (performMainRobotAction(action))
            {
                robot->logger_ << "[Strategy (main_robot)] " << "Action was successful: removed" << std::endl;
                actions.erase( actions.begin() + action_index);
            }
            else
            {
                robot->logger_ << "[Strategy (main_robot)] " << "Action was not successful: deactivated" << std::endl;
                action->activated = false;
            }
        }

        if (number_of_unsuccessful_iters > 10)
        {
            // robot->logger_ << "[Strategy (main_robot)] " << "Removing all actions" << std::endl;
            // actions.clear();
            robot->logger_ << "[Strategy (main_robot)] " << "Go back in the middle" << std::endl;
            RobotPosition position;
            position.x = 1000;
            position.y = 1500;
            position.theta = -M_PI_2;

            if ((motionController->getCurrentPosition() - position).norm() > 100)
            {
                traj = robot->getMotionController()->computeMPCTrajectory(position, robot->getMotionController()->getDetectedObstacles(), true);
                if (!traj.empty())
                {
                    robot->getMotionController()->setTrajectoryToFollow(traj);
                    robot->getMotionController()->waitForTrajectoryFinished();
                }
            }
        }
    }

    // Match end: go back to base
    goBackToBase();
    // MATCH_COMPLETED = true;

    while (true) ;;

    robot->logger_ << "[Strategy (main_robot)] " << "Strategy thread ended" << robot->getMatchTime() << std::endl;

}
}
