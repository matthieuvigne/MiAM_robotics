/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/PathPlanner.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include "main_robot/Strategy.h"
#include "common/DH_transform.hpp"
#include "common/MotionPlanner.h"
#include "common/ArmInverseKinematics.hpp"
#include "common/ThreadHandler.h"

#include "main_robot/PickupPlantsAction.h"
#include "main_robot/DropPlantsAction.h"

using namespace miam::trajectory;
using miam::RobotPosition;
using namespace kinematics;

namespace main_robot
{

bool MATCH_COMPLETED = false;


//--------------------------------------------------------------------------------------------------

Strategy::Strategy()
{
    // Empty on purpose*
}

//--------------------------------------------------------------------------------------------------

bool Strategy::setup(RobotInterface *robot)
{
    // Get robot
    this->robot = robot;
    this->motionController = robot->getMotionController();
    robot->logger_ << "[Strategy] Strategy setup is being performed" << std::endl;
    servoManager_.init(robot);


    motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_MPC);

    // Load actions into action vector.
    actions_.clear();

    for (int i = 0; i < 6; i++)
    {
        actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, i));
        actions_.push_back(std::make_shared<DropPlantsAction>(robot, &servoManager_, i));
    }

    return true;
}

//--------------------------------------------------------------------------------------------------

void Strategy::shutdown()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

// Action* Strategy::chooseNextAction(
//     std::vector<Action>& actions,
//     RobotPosition currentPosition,
//     MotionPlanner& motionPlanner
// )
// {

//     Action* bestAction = &actions.front();
//     double bestLoss = 0;

//     int i = 0;

//     // if all actions are deactivated, reactivate all actions
//     bool all_actions_deactivated = true;
//     for (auto & a : actions)
//     {
//         if (a.isActivated_)
//             all_actions_deactivated = false;
//     }
//     if (all_actions_deactivated)
//     {
//         std::cout << "All actions are deactivated: reactivate all" << std::endl;
//         for (auto & a : actions)
//             a.isActivated_ = true;
//     }

//     for (auto & a : actions)
//     {
//         if (a.isActivated_)
//         {
//             // compute loss
//             double timeItTakesToGoThere = motionPlanner.computeMotionTime(robot->getParameters().getTrajConf(), currentPosition, a.startPosition_);
//             double currentLoss = a.nPointsToGain_ /(a.timeItTakes_ + timeItTakesToGoThere);

//             std::cout << "Calcul pour " << i << " : " << timeItTakesToGoThere << ", " << currentLoss << std::endl;

//             if (bestLoss < currentLoss)
//             {
//                 bestAction = &a;
//                 bestLoss = currentLoss;
//                 std::cout << "Chosen action " << i << std::endl;
//             }
//         }
//         i++;
//     }

//     return bestAction;
// }

//--------------------------------------------------------------------------------------------------

void Strategy::match()
{
    robot->logger_ << "Strategy thread started." << std::endl;

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = ThreadHandler::addThread(stratMain);

    double const FALLBACK_TIME = 85.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    usleep(50000);
    if (!MATCH_COMPLETED)
    {
        robot->logger_ << "Match almost done, auto-triggering fallback strategy" << std::endl;
    }
    // TODO: don't go there if you already are
    robot->setGUIActionName("[Match End] Back to base");
    goBackToBase();
}


void Strategy::goBackToBase()
{
  RobotPosition targetPosition{825,550,0}; // 700
  go_to_straight_line(targetPosition, 1.5, false);
  targetPosition.y -= 350; // 500;
  go_to_straight_line(targetPosition, 1.5, false);
}


//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
    int highestPriorityAction = 1;
    while (highestPriorityAction > 0)
    {
        highestPriorityAction = -1;

        robot->logger_ << "[Strategy] Choosing action from list: " << std::endl;
        for (unsigned int i = 0; i < actions_.size(); i++)
        {
            actions_.at(i)->updateStartCondition();
            if (actions_.at(i)->priority_ > highestPriorityAction)
                highestPriorityAction = actions_.at(i)->priority_;
            robot->logger_ << i << "\t" << *actions_.at(i) << std::endl;
        }

        // Look amongst action with highest priority, select the closest one.
        int selectedAction = 0;
        double minDistance = 1e10;
        RobotPosition currentPosition = motionController->getCurrentPosition();
        for (unsigned int i = 0; i < actions_.size(); i++)
        {
            if (actions_.at(i)->priority_ == highestPriorityAction)
            {
                double const distance = (actions_.at(i)->startPosition_ - currentPosition).norm();
                if (distance < minDistance)
                {
                    minDistance = distance;
                    selectedAction = i;
                }
            }
        }


        // Perform this action
        robot->logger_ << "[Strategy] Chose action " << selectedAction << std::endl;
        robot->setGUIActionName(actions_.at(selectedAction)->getName());

        bool actionNeedsToBeRemoved = performAction(actions_.at(selectedAction));
        robot->setGUIActionName("None");

        if (actionNeedsToBeRemoved)
        {
            actions_.erase(actions_.begin() + selectedAction);
        }
    }
    robot->logger_ << "[Strategy] No more action to perform" << std::endl;


}

bool Strategy::performAction(std::shared_ptr<AbstractAction> action)
{
    robot->logger_ << "[Strategy] Performing action: " << *action << std::endl;
    // Go to the start of the action
    TrajectoryVector traj;
    traj = robot->getMotionController()->computeMPCTrajectory(action->startPosition_, robot->getMotionController()->getDetectedObstacles(), true);
    if (traj.empty())
    {
        robot->logger_ << "[Strategy] Motion planning to action failed!" << std::endl;
        return false;
    }

    // Perform trigger action
    action->actionStartTrigger();

    // Follow trajectory to action start
    robot->getMotionController()->setTrajectoryToFollow(traj);
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

    return true;
}


}
