/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "main_robot/Strategy.h"

using namespace miam::trajectory;
using miam::RobotPosition;


#define USE_CAMERA 1

namespace main_robot
{

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;


Strategy::Strategy()
{
  // [TODO]
}

void Strategy::setup(RobotInterface *robot)
{
    // Get robot
    this->robot = robot;
    this->servo = robot->getServos();
    this->motionController = robot->getMotionController();

    // Set initial position
    RobotPosition targetPosition;
    targetPosition.x = 2000 - robot->getParameters().CHASSIS_BACK;
    targetPosition.y = 2000 - robot->getParameters().CHASSIS_WIDTH/2.0;
    targetPosition.theta = M_PI;
    motionController->resetPosition(targetPosition, true, true, true);
}


Action* Strategy::chooseNextAction(
    std::vector<Action>& actions,
    RobotPosition currentPosition,
    MotionPlanning motionPlanner
)
{

    Action* bestAction = &actions.front();
    double bestLoss = 0;

    int i = 0;

    // if all actions are deactivated, reactivate all actions
    bool all_actions_deactivated = true;
    for (auto & a : actions)
    {
        if (a.isActivated_)
            all_actions_deactivated = false;
    }
    if (all_actions_deactivated)
    {
        std::cout << "All actions are deactivated: reactivate all" << std::endl;
        for (auto & a : actions)
            a.isActivated_ = true;
    }

    for (auto & a : actions)
    {
        if (a.isActivated_)
        {
            // compute loss
            double timeItTakesToGoThere = motionPlanner.computeMotionTime(robot->getParameters().getTrajConf(), currentPosition, a.startPosition_);
            double currentLoss = a.nPointsToGain_ /(a.timeItTakes_ + timeItTakesToGoThere);

            std::cout << "Calcul pour " << i << " : " << timeItTakesToGoThere << ", " << currentLoss << std::endl;

            if (bestLoss < currentLoss)
            {
                bestAction = &a;
                bestLoss = currentLoss;
                std::cout << "Chosen action " << i << std::endl;
            }
        }
        i++;
    }

    return bestAction;
}

void Strategy::match_impl()
{

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    
    // Set initial position
    targetPosition.x = 2000 - robot->getParameters().CHASSIS_BACK;
    targetPosition.y = 2000 - robot->getParameters().CHASSIS_WIDTH/2.0;
    targetPosition.theta = M_PI;
    motionController->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    // Create brain
    MotionPlanning motion_planner;
     //~ action1(0, 1, RobotPosition(1275+robot->getParameters().CHASSIS_FRONT,1875,0));
    //~ Action action2(0, 1, RobotPosition(345+robot->getParameters().CHASSIS_FRONT,2325,0));
    //~ Action action3(0, 1, RobotPosition(120+robot->getParameters().CHASSIS_FRONT,2325,0));
    //~ Action action3(0, 1, RobotPosition(120+robot->getParameters().CHASSIS_FRONT,2325,0));

    std::vector<Action> actionVector;
    actionVector.push_back(Action(0, 1, RobotPosition(1275+robot->getParameters().CHASSIS_FRONT,1875,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(345+robot->getParameters().CHASSIS_FRONT,2325,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(120+robot->getParameters().CHASSIS_FRONT,2325,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(225,2775-3*120,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(450+275,1875,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(1775-robot->getParameters().CHASSIS_FRONT,2275,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(2000-120-robot->getParameters().CHASSIS_FRONT,2275,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(2000-225,1500+225+375+350-3*120,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(2000-225,1500+225+375+350-3*120+20,0)));
    actionVector.push_back(Action(0, 1, RobotPosition(225,450+125+200+125+225,0)));
    // [TODO] Assemblage gateau
    //~ actionVector.push_back(action3);



    while (!actionVector.empty())
    {

        std::cout << ">>> RE-ASSESSING ACTIONS" << std::endl;

        std::cout << "Remaining actions : " << actionVector.size() << std::endl;
        for (auto v : actionVector)
        {
            std::cout << v << std::endl;
        }

        Action* nextAction = chooseNextAction(actionVector, motionController->getCurrentPosition(), motion_planner);

        std::cout << ">>> nextAction : " << *nextAction << std::endl;

        targetPosition = motionController->getCurrentPosition();
        traj = motion_planner.computeTraj(robot->getParameters().getTrajConf(), targetPosition, nextAction->startPosition_);
        // Disable avoidance?
        // for (auto i : traj)
        // {
        //     i->setAvoidanceEnabled(false);
        // }
        motionController->setTrajectoryToFollow(traj);
        bool moveSuccess = motionController->waitForTrajectoryFinished();

        if (moveSuccess)
        {
            nextAction->performAction(robot);
            actionVector.erase(std::find(actionVector.begin(), actionVector.end(), *nextAction));

            // Reactivate all actions
            for (auto & a : actionVector)
                a.isActivated_ = true;
        }
        else
        {
            // Penaliser l'action
            nextAction->isActivated_ = false;
        }

    }



    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = stratMain.native_handle();
    createdThreads_.push_back(handle);
    stratMain.detach();

    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    usleep(50000);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;

        servo->activatePump(false);

        //~ if (goBackToDigSite())
        //~ {
            //~ robot->updateScore(20); //match completed
            //~ camera_.shutDown();
        //~ }
    }
}


}
