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
    if (isSetupFirstInstance_)
    {
        isSetupFirstInstance_ = false;
        // Get robot
        this->robot = robot;
        this->motionController = robot->getMotionController();
        robot->logger_ << "[Strategy] Strategy setup is being performed" << std::endl;

        // Init servo manager, this starts turret calibration
        servoManager_.init(robot);


        motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_MPC);

        // Load actions into action vector.
        actions_.clear();

        for (int i = 0; i < 6; i++)
        {
            actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, i));
            actions_.push_back(std::make_shared<DropPlantsAction>(robot, &servoManager_, i));
        }
    }

    // Wait until turret is calibrating.
    return servoManager_.isTurretMotionless();
}

//--------------------------------------------------------------------------------------------------

void Strategy::shutdown()
{
  // TODO
}

void Strategy::match()
{
    robot->logger_ << "Strategy thread started." << std::endl;

    testSquare(true, 500);
    return;

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
    // Target depends on start position
    RobotPosition targetPosition;
    if (robot->getStartPosition().y < 700)
    {
        targetPosition.x = 250;
        targetPosition.y = 1750;
    }
    else
    {
        targetPosition.x = 250;
        targetPosition.y = 250;
    }
    go_to_straight_line(targetPosition);
}


//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
    int highestPriorityAction = 1;
    while (highestPriorityAction > 0 && !actions_.empty())
    {
        highestPriorityAction = -1;

        robot->logger_ << "[Strategy] Choosing action from list: " << std::endl;

        for (unsigned int i = 0; i < actions_.size(); i++)
        {
            actions_.at(i)->updateStartCondition();
            if (!actions_.at(i)->wasFailed && actions_.at(i)->priority_ > highestPriorityAction)
                highestPriorityAction = actions_.at(i)->priority_;
            robot->logger_ << i << "\t" << *actions_.at(i) << std::endl;
        }

        // Look amongst action with highest priority, select the closest one.
        int selectedAction = 0;
        double minDistance = 1e10;
        RobotPosition currentPosition = motionController->getCurrentPosition();
        for (unsigned int i = 0; i < actions_.size(); i++)
        {
            if (actions_.at(i)->priority_ == highestPriorityAction && !actions_.at(i)->wasFailed)
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

        bool actionShouldBeRemoved;
        bool actionSuccessful = performAction(actions_.at(selectedAction), actionShouldBeRemoved);

        // If action failed: don't try it again for now until one is successful
        if (actionSuccessful)
        {
            for (unsigned int i = 0; i < actions_.size(); i++)
            {
                actions_.at(i)->wasFailed = false;
            }
        }
        else
        {
            actions_.at(selectedAction)->wasFailed = true;
            // If all actions have failed, let's try everything over again.
            bool allFail = true;
            for (unsigned int i = 0; i < actions_.size(); i++)
            {
                allFail = allFail & actions_.at(i)->wasFailed;
            }
            if (allFail)
            {
                robot->logger_ << "[Strategy] All actions have failed, let's reset." << std::endl;
                for (unsigned int i = 0; i < actions_.size(); i++)
                {
                    actions_.at(i)->wasFailed = false;
                }
            }
        }
        robot->setGUIActionName("None");

        if (actionShouldBeRemoved)
        {
            actions_.erase(actions_.begin() + selectedAction);
        }
    }
    robot->logger_ << "[Strategy] No more action to perform" << std::endl;


}

bool Strategy::performAction(std::shared_ptr<AbstractAction> action, bool & actionShouldBeRemoved)
{
    actionShouldBeRemoved = false;
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
    actionShouldBeRemoved = action->performAction();
    return true;
}


}
