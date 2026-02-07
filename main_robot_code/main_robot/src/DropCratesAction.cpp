#include "main_robot/DropCratesAction.h"

#define MARGIN 130
#define BUILD_DISTANCE 170


void DropCratesAction::updateStartCondition()
{
    // TODO do not drop if zone is full
    if (robot_->getGameState()->isPantryZoneUsed[zoneId_])
    {
        priority_ = -1;
    }
    else if (robot_->getGameState()->isRobotFull && robot_->getGameState()->isClawFull)
    {
        priority_ = 10;
        if (zoneId_ == 1)
            priority_ += 2;
        if (zoneId_ == 0)
            priority_ += 5;
    }
    else if (!robot_->getGameState()->isRobotFull && !robot_->getGameState()->isClawFull)
    {
        priority_ = -1;
    }
    else if (robot_->getMatchTime() > 60.0)
    {
        priority_ = 6;
    }

    startPosition_ = PANTRY_ZONE_COORDS[zoneId_];
}


void DropCratesAction::actionStartTrigger()
{
    // Empty on purpose
}

bool DropCratesAction::performAction()
{
    robot_->logger_ << "[DropCratesAction] Starting action " << zoneId_ << std::endl;
    int const sign = (isStartMotionBackward_ ? -1 : 1);

    // Move the cans in the planned spot.
    robot_->getMotionController()->goStraight(sign * MARGIN);

    bool dropped_something = false;
    if (robot_->getGameState()->isClawFull)
    {
        // TODO drop claw
        robot_->getGameState()->isClawFull = false;
        dropped_something = true;
    }
    else if (robot_->getGameState()->isRobotFull)
    {
        // TODO drop inside
        robot_->getGameState()->isRobotFull = false;
        dropped_something = true;
    }
    if (dropped_something)
    {
        robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;
        robot_->updateScore(4, "dropped something");

    }

    return true;
}


