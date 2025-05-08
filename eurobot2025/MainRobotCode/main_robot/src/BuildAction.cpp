#include "main_robot/BuildAction.h"

#define BACK_EXTRA_OFFSET 48


void BuildAction::updateStartCondition()
{
    if (robot_->getGameState()->isFrontClawFull && robot_->getGameState()->isBackClawFull)
    {
        priority_ = 10;
        if (zoneId_ == 2)
            priority_ += 5;
        if (zoneId_ == 0)
            priority_ += 2;
    }
    else if (!robot_->getGameState()->isFrontClawFull && !robot_->getGameState()->isBackClawFull)
    {
        priority_ = -1;
    }
    else if (robot_->getMatchTime() > 60.0)
    {
        priority_ = 6;
    }

    startPosition_ = CONSTRUCTION_ZONE_COORDS[zoneId_].forward(-300);

    if (!robot_->getGameState()->isFrontClawFull)
        isStartMotionBackward_ = true;
    else if (robot_->getGameState()->isBackClawFull && (zoneId_ == 0 || zoneId_ == 2))
        isStartMotionBackward_ = true;
    if (isStartMotionBackward_)
        startPosition_.theta += M_PI;
}


void BuildAction::actionStartTrigger()
{
    // Empty on purpose
}

bool BuildAction::performAction()
{
    int const sign = (isStartMotionBackward_ ? -1 : 1);

    robot_->getMotionController()->goStraight(sign * (100 + BACK_EXTRA_OFFSET));
    if (isStartMotionBackward_)
    {
        servoManager_->dropBackCans();
        robot_->getGameState()->isBackClawFull = false;
    }
    else
    {
        servoManager_->buildFrontTower();
        robot_->getGameState()->isFrontClawFull = false;
    }

    robot_->getGameState()->isConstructionZoneUsed[zoneId_] = true;

    if (isStartMotionBackward_ && robot_->getGameState()->isFrontClawFull && (zoneId_ == 0 || zoneId_ == 2))
    {
        // Build front tower, put it on top of the other one
        robot_->getMotionController()->goStraight(300);
        robot_->getMotionController()->pointTurn(M_PI);
        robot_->getMotionController()->goStraight(150);
        servoManager_->buildFrontTower();
        robot_->getGameState()->isFrontClawFull = false;

        robot_->getMotionController()->goStraight(-250);
        robot_->getMotionController()->pointTurn(M_PI);
        robot_->getMotionController()->goStraight(-(250 + BACK_EXTRA_OFFSET));
        servoManager_->grab(false);
        servoManager_->backRail_.move(1.0);
        while (servoManager_->backRail_.isMoving())
            robot_->wait(0.050);
        robot_->getMotionController()->goStraight(-150 + BACK_EXTRA_OFFSET, 0.5);

        servoManager_->dropBackCans(false);
        robot_->getMotionController()->goStraight(200);
    }
    else
        robot_->getMotionController()->goStraight(-sign * 200);

    // Action should not be done again
    return true;
}


