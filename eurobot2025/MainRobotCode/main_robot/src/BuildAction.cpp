#include "main_robot/BuildAction.h"

#define BACK_EXTRA_OFFSET 48

#define MARGIN 200


void BuildAction::updateStartCondition()
{
    if (robot_->getGameState()->isFrontClawFull && robot_->getGameState()->isBackClawFull)
    {
        priority_ = 10;
        if (zoneId_ == 1)
            priority_ += 2;
        if (zoneId_ == 0)
            priority_ += 5;
    }
    else if (!robot_->getGameState()->isFrontClawFull && !robot_->getGameState()->isBackClawFull)
    {
        priority_ = -1;
    }
    else if (robot_->getMatchTime() > 60.0)
    {
        priority_ = 6;
    }

    // Default to back drop on large zones, front drop on small zones
    if (largeZone_)
        isStartMotionBackward_ = robot_->getGameState()->isBackClawFull;
    else
        isStartMotionBackward_ = !robot_->getGameState()->isFrontClawFull;

    double const xoffset = (isStartMotionBackward_ ? BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET) + MARGIN;
    startPosition_ = CONSTRUCTION_ZONE_COORDS[zoneId_].forward(-xoffset - 230 * nDrop_);

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

    robot_->getMotionController()->goStraight(sign * MARGIN);
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

    if (isStartMotionBackward_ && robot_->getGameState()->isFrontClawFull && (largeZone_))
    {
        // Build front tower, put it on top of the other one
        robot_->getMotionController()->goStraight(2 * MARGIN);
        robot_->getMotionController()->pointTurn(M_PI);
        robot_->getMotionController()->goStraight(MARGIN);
        servoManager_->buildFrontTower();
        robot_->getGameState()->isFrontClawFull = false;

        robot_->getMotionController()->goStraight(-MARGIN);

        /*
        Disabled for now: raise column
        robot_->getMotionController()->pointTurn(M_PI);
        robot_->getMotionController()->goStraight(-(MARGIN + BACK_DIFF_XOFFSET));
        servoManager_->grab(false);
        servoManager_->backRail_.move(1.0);
        while (servoManager_->backRail_.isMoving())
            robot_->wait(0.050);
        robot_->getMotionController()->goStraight(-MARGIN, 0.5);

        servoManager_->dropBackCans(false);
        robot_->getMotionController()->goStraight(MARGIN);
        */
    }
    else
        robot_->getMotionController()->goStraight(-sign * 200);

    // Allow several drops in large zones
    if (largeZone_)
    {
        nDrop_ ++;
        return nDrop_ > 0;
        // return nDrop_ > 1;
    }
    return true;
}


