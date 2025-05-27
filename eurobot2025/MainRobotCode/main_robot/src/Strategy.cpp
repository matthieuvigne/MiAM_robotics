#include "main_robot/Strategy.h"
#include "common/MotionPlanner.h"
#include "common/ThreadHandler.h"

#include "main_robot/GrabColumnAction.h"
#include "main_robot/BuildAction.h"
#include "main_robot/SmallColumnAction.h"

using namespace miam::trajectory;
using miam::RobotPosition;

namespace main_robot
{

bool MATCH_COMPLETED = false;


//--------------------------------------------------------------------------------------------------

Strategy::Strategy(bool const& interactive):
    interactive_(interactive)
{
    // Empty on purpose*
}

//--------------------------------------------------------------------------------------------------

bool Strategy::setup(RobotInterface *robot)
{
    if (setupStep_ == 0)
    {
        setupStep_ ++;
        // Get robot
        this->robot = robot;
        this->motionController = robot->getMotionController();
        robot->logger_ << "[Strategy] Strategy setup is being performed" << std::endl;

        // Init servo manager, this starts turret calibration
        servoManager_.init(robot);

        motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_MPC);

        // Load actions into action vector.
        actions_.clear();

        // Zone 0 disabled to protect PAMI
        for (int i = 1; i < 8; i++)
        {
            actions_.push_back(std::make_shared<GrabColumnAction>(robot, &servoManager_, i));
        }

        // Build action - no zone 2, this is handled by the SmallColumn action
        actions_.push_back(std::make_shared<BuildAction>(robot, &servoManager_, 0));
        actions_.push_back(std::make_shared<BuildAction>(robot, &servoManager_, 1));
        actions_.push_back(std::make_shared<BuildAction>(robot, &servoManager_, 3));

        actions_.push_back(std::make_shared<SmallColumnAction>(robot, &servoManager_));

    }
    if (setupStep_ == 1 && servoManager_.isRailCalibDone())
    {
        servoManager_.setRailsToInitPosition();
        setupStep_ = 2;
        #ifdef SIMULATION
        return true;
        #endif
    }
    if (setupStep_ == 2 && !servoManager_.railManager_.areAnyMoving())
    {
        servoManager_.frontRightClaw_.move(ClawPosition::FOLDED);
        servoManager_.frontLeftClaw_.move(ClawPosition::FOLDED);
        servoManager_.foldPlank();
        servoManager_.frontRightClaw_.foldClaw();
        servoManager_.frontLeftClaw_.foldClaw();
        servoManager_.foldBackPlank();
        return true;
    }
    return false;
}

//--------------------------------------------------------------------------------------------------

void Strategy::shutdown()
{
  // TODO
}

void Strategy::match()
{
    pthread_setname_np(pthread_self(), "strat_match");
    robot->logger_ << "Strategy thread started." << std::endl;


    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = ThreadHandler::addThread(stratMain);
    createdThreads_.push_back(handle);

    // Disable fallback, action system does it by itself.
    double const FALLBACK_TIME = 100.0;
    robot->wait(FALLBACK_TIME);
    // if (!MATCH_COMPLETED)
    // {
    //     pthread_cancel(handle);
    //     usleep(50000);
    //     robot->logger_ << "Match almost done, auto-triggering fallback strategy" << std::endl;
    //     goBackToBase();
    // }
    // else
    // {
    //     robot->wait(100.0 - FALLBACK_TIME);
    //     pthread_cancel(handle);
    // }
    // robot->wait(100.0 - robot->getMatchTime());
    servoManager_.railManager_.abort();
}


void Strategy::goBackToBase()
{
    MATCH_COMPLETED = true;
    robot->getMotionController()->enableDetection(true);
    robot->setGUIActionName("[Match End] Back to base");
    // Clear current trajectory
    robot->getMotionController()->stopCurrentTrajectoryTracking();

    // Release everything
    servoManager_.backClawOpen();
    servoManager_.frontClawOpen();
    servoManager_.releasePlank();
    servoManager_.releaseBackPlank();

    servoManager_.frontLeftClaw_.rail_.move(0.95);
    servoManager_.frontRightClaw_.rail_.move(0.95);
    servoManager_.frontRightClaw_.move(ClawPosition::FORWARD);
    servoManager_.frontLeftClaw_.move(ClawPosition::FORWARD);
    servoManager_.frontRightClaw_.openClaw();
    servoManager_.frontLeftClaw_.openClaw();

    robot->getGameState()->isFrontClawFull = false;
    robot->getGameState()->isBackClawFull = false;

    // Target depends on start position
    RobotPosition targetPosition(300, 1400, M_PI);

    bool targetReached = false;
    while (!targetReached)
    {

        TrajectoryVector traj;
        traj = robot->getMotionController()->computeMPCTrajectory(
            targetPosition,
            robot->getMotionController()->getDetectedObstacles(),
            tf::DEFAULT);
        if (!traj.empty())
        {
            robot->getMotionController()->setTrajectoryToFollow(traj);
            targetReached = robot->getMotionController()->waitForTrajectoryFinished();
        }
        // No fallback this time
        // else
        //     targetReached = robot->getMotionController()->goToStraightLine(targetPosition);
    }
    if (targetReached)
    {
        servoManager_.foldClaws();
        robot->updateScore(10, "back to base");
        if (robot->isPlayingRightSide())
        {
            servoManager_.frontLeftClaw_.move(ClawPosition::SIDE);
            servoManager_.frontLeftClaw_.foldClaw();
        }
        else
        {
            servoManager_.frontRightClaw_.move(ClawPosition::SIDE);
            servoManager_.frontRightClaw_.foldClaw();
        }
    }
}


//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
    pthread_setname_np(pthread_self(), "strat_matchImpl");

    servoManager_.dropBanner();
    robot->wait(0.5);
    robot->updateScore(20, "banner");

    SmallColumnAction act(robot, &servoManager_);
    act.updateStartCondition();
    RobotPosition current = robot->getMotionController()->getCurrentPosition();
    RobotPosition final = act.startPosition_;

    std::vector<RobotPosition> positions;
    current.y = final.y;
    positions.push_back(current);
    positions.push_back(final);
    robot->getMotionController()->goToRoundedCorners(positions, 200, 0.3);
    servoManager_.foldBanner();
    servoManager_.releasePlank();


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

        if (robot->getMatchTime() > 82.0)
        {
            robot->logger_ << "[Strategy] Near match end, let's go back." << std::endl;
            break;
        }
    }
    robot->logger_ << "[Strategy] No more action to perform" << std::endl;
    goBackToBase();
}

bool Strategy::performAction(std::shared_ptr<AbstractAction> action, bool & actionShouldBeRemoved)
{
    actionShouldBeRemoved = false;
    if (action->priority_ < 0)
    {
        robot->logger_ << "[Strategy] Action aborted, negative priority." << *action << std::endl;
        return false;
    }
    robot->logger_ << "[Strategy] Performing action: " << *action << std::endl;
    robot->getMotionController()->enableDetection(true);
    // Go to the start of the action
    tf flags = (action->isStartMotionBackward_ ? tf::BACKWARD : tf::DEFAULT);
    if (action->ignoreFinalRotation_)
        flags = static_cast<tf>(flags | tf::IGNORE_END_ANGLE);

    // Perform trigger action
    action->actionStartTrigger();

    // Only do pathplanning if not there yet
    if (!(action->ignoreFinalRotation_ && (robot->getMotionController()->getCurrentPosition() - action->startPosition_).norm() < 20))
    {
        TrajectoryVector traj;
        traj = robot->getMotionController()->computeMPCTrajectory(
            action->startPosition_,
            robot->getMotionController()->getDetectedObstacles(),
            flags);

        // Second attempt assuming obstacle is square
        if (traj.empty())
        {
            traj = robot->getMotionController()->computeMPCTrajectory(
            action->startPosition_,
            robot->getMotionController()->getDetectedObstacles(),
            flags,
            0,
            false);
        }

        if (traj.empty())
        {
            robot->logger_ << "[Strategy] Motion planning to action failed!" << std::endl;
            return false;
        }


        // Follow trajectory to action start
        robot->getMotionController()->setTrajectoryToFollow(traj);
        if (!robot->getMotionController()->waitForTrajectoryFinished())
        {
            robot->logger_ << "[Strategy] Action failed, could not reach start position." << std::endl;
            // action was not successful
            return false;
        }
    }
    actionShouldBeRemoved = action->performAction();
    return actionShouldBeRemoved;
}


}
