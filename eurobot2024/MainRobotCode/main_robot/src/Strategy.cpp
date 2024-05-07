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
#include "main_robot/SolarPanelsAction.h"

using namespace miam::trajectory;
using miam::RobotPosition;
using namespace kinematics;

namespace main_robot
{

bool MATCH_COMPLETED = false;


//--------------------------------------------------------------------------------------------------

Strategy::Strategy(bool const& interactive, bool const& isTurretAlreadyCalibrated):
    interactive_(interactive),
    isTurretAlreadyCalibrated_(isTurretAlreadyCalibrated)
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
        servoManager_.init(robot, isTurretAlreadyCalibrated_);


        motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_MPC);

        // Load actions into action vector.
        actions_.clear();

        // Ignore the two rightmost zones.
        actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, 0));
        actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, 1));
        actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, 3));
        actions_.push_back(std::make_shared<PickupPlantsAction>(robot, &servoManager_, 5));

        //~ actions_.push_back(std::make_shared<DropPlantsWithoutPotsAction>(robot, &servoManager_, 0));
        actions_.push_back(std::make_shared<DropPlantsWithPotAction>(robot, &servoManager_, 0));
        actions_.push_back(std::make_shared<DropPlantsWithPotAction>(robot, &servoManager_, 1));
        actions_.push_back(std::make_shared<DropPlantsWithoutPotsAction>(robot, &servoManager_, 2));

        for (int i = 0; i < 3; i++)
        {
            actions_.push_back(std::make_shared<DropPlantsToJarnidiereAction>(robot, &servoManager_, i));
        }

        actions_.push_back(std::make_shared<SolarPanelsAction>(robot, &servoManager_,
            SOLAR_PANEL_SIDE|SOLAR_PANEL_CENTER));
        //~ actions_.push_back(std::make_shared<SolarPanelsAction>(robot, &servoManager_, SOLAR_PANEL_CENTER));
    }

    // Wait until turret is calibrating.
    bool done = servoManager_.isTurretMotionless();
    if (done)
    {
        servoManager_.setClawPosition(ClawSide::FRONT, ClawPosition::START_POSITION);
        servoManager_.setClawPosition(ClawSide::BACK, ClawPosition::START_POSITION);
    }
    return done;
}

//--------------------------------------------------------------------------------------------------

void Strategy::shutdown()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

void Strategy::match()
{
    pthread_setname_np(pthread_self(), "strat_match");
    robot->logger_ << "Strategy thread started." << std::endl;


    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = ThreadHandler::addThread(stratMain);
    createdThreads_.push_back(handle);

    double const FALLBACK_TIME = 82.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    usleep(50000);
    if (!MATCH_COMPLETED)
    {
        robot->logger_ << "Match almost done, auto-triggering fallback strategy" << std::endl;
    }
    robot->setGUIActionName("[Match End] Back to base");
    goBackToBase();
}

//--------------------------------------------------------------------------------------------------

void Strategy::goBackToBase()
{
    // Clear current trajectory
    robot->getMotionController()->stopCurrentTrajectoryTracking();

    // Close all available claws
    bool isFront = std::abs(servoManager_.getTurretPosition()) < 0.1;
    servoManager_.closeClaws(isFront);

    // Target depends on start position
    RobotPosition targetPosition;
    if (robot->getStartPosition().y < 700)
    {
        targetPosition.x = 450;
        targetPosition.y = 1600;
    }
    else
    {
        targetPosition.x = 450;
        targetPosition.y = 400;
    }
    targetPosition.theta = M_PI;
    servoManager_.stopSolarPanel();
    servoManager_.raiseSolarPanelArm();
    robot->getMPC23008()->setOutputs(0);

    // Prepare to drop plants
    isFront = robot->gameState_.nPlantsInClaw(true) > 0;
    if (robot->gameState_.nPlantsInRobot() > 0)
        servoManager_.moveTurret(isFront ? 0 : M_PI);
    else
    {
        servoManager_.setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
        servoManager_.setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    }

    RobotPosition targetPositions[2] = {targetPosition, RobotPosition(2700, 970, M_PI_2)};
    int candidateId = 1;
    bool targetReached = false;
    while (!targetReached && robot->getMatchTime() < 90)
    {

        TrajectoryVector traj;
        traj = robot->getMotionController()->computeMPCTrajectory(
            targetPositions[candidateId],
            robot->getMotionController()->getDetectedObstacles(),
            tf::DEFAULT);
        if (!traj.empty())
        {
            robot->getMotionController()->setTrajectoryToFollow(traj);
            targetReached = robot->getMotionController()->waitForTrajectoryFinished();
        }
        else
            targetReached = robot->getMotionController()->goToStraightLine(targetPositions[candidateId]);
        candidateId  = (candidateId + 1) % 2;
    }
    if (targetReached)
        robot->updateScore(10, "back to base");

    if (robot->gameState_.nPlantsInRobot() > 0)
    {
        dropPlants(robot, &servoManager_, isFront, 0, true);
    }
    //~ servoManager_.openClaws(true);
    //~ servoManager_.openClaws(false);
    robot->getMotionController()->goStraight(-120);

    // No final motion so we don't kill PAMI.
    if (robot->gameState_.nPlantsInRobot() > 0 || true)
    {
      servoManager_.setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);
      robot->wait(1.0);
      isFront = !isFront;
      servoManager_.moveTurret(isFront ? 0 : M_PI);
      robot->wait(1.0);
      dropPlants(robot, &servoManager_, isFront, 0, true);
    }
}


//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
    pthread_setname_np(pthread_self(), "strat_matchImpl");
    // vision::SolarPanelCamera camera("/dev/video0");
    // while (true)
    // {
    //     double angle_deg = camera.getSolarPanelOrientation(true);
    //     std::cout << angle_deg << std::endl;
    // }
    // testSquare();
    // robot->getMotionController()->goStraight(-600);
    // while (true) ;;

    // // robot->getMotionController()->pointTurn(M_PI);
    // robot->getMotionController()->pointTurn(-M_PI + 1e-6);
    // robot->wait(1.0);
    // RobotPosition pos;
    // pos.theta = 0;
    // robot->getMotionController()->resetPosition(pos, false, false, true);
    // robot->getMotionController()->pointTurn(M_PI);
    // robot->getMotionController()->pointTurn(-M_PI + 1e-6);
    // while (true) ;;

    while (!actions_.empty())
    {
        int highestPriorityAction = -1;

        robot->logger_ << "[Strategy] Choosing action from list: " << std::endl;

        for (unsigned int i = 0; i < actions_.size(); i++)
        {
            actions_.at(i)->updateStartCondition();
            if (!actions_.at(i)->wasFailed && actions_.at(i)->priority_ > highestPriorityAction)
                highestPriorityAction = actions_.at(i)->priority_;
            robot->logger_ << i << "\t" << *actions_.at(i) << std::endl;
        }

        int selectedAction = 0;

        if (interactive_)
        {
            // Let the user choose the action
            std::string userInput;
            std::cout << "Type action number to perform" << std::endl;
            std::cin >> userInput;
            selectedAction = std::stoi(userInput);
        }
        else
        {
            // Look amongst action with highest priority, select the closest one.
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
        }


        // Perform this action
        robot->logger_ << "[Strategy] Chose action " << selectedAction << std::endl;
        robot->setGUIActionName(actions_.at(selectedAction)->getName());

        bool actionShouldBeRemoved;
        bool actionSuccessful = performAction(actions_.at(selectedAction), actionShouldBeRemoved);

        // After each action: raise arms, we never want to transition with low arms.
        servoManager_.setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
        servoManager_.setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);


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
    traj = robot->getMotionController()->computeMPCTrajectory(
        action->startPosition_,
        robot->getMotionController()->getDetectedObstacles(),
        (action->isStartMotionBackward_ ? tf::BACKWARD : tf::DEFAULT));
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
        robot->logger_ << "[Strategy] Action failed, could not reach start position." << std::endl;
        // action was not successful
        return false;
    }
    actionShouldBeRemoved = action->performAction();
    return actionShouldBeRemoved;
}


}
