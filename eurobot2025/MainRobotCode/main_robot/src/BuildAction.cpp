#include "main_robot/BuildAction.h"

void BuildAction::updateStartCondition()
{
    if (robot_->gameState_.isFrontClawFull && robot_->gameState_.isBackClawFull)
    {
        priority_ = 10;
    }
    else if (!robot_->gameState_.isFrontClawFull && !robot_->gameState_.isBackClawFull)
    {
        priority_ = -1;
    }
    else if (robot_->getMatchTime() > 60.0)
    {
        priority_ = 6;
    }

    startPosition_ = CONSTRUCTION_ZONE_COORDS[zoneId_].forward(-300);
}


void BuildAction::actionStartTrigger()
{
    // TODO
}

bool BuildAction::performAction()
{
    robot_->getMotionController()->goStraight(100);
    servoManager_->buildFrontTower();
    robot_->getMotionController()->goStraight(-200);


    robot_->gameState_.isConstructionZoneUsed[zoneId_] = true;

    robot_->gameState_.isFrontClawFull = false;
    robot_->gameState_.isBackClawFull = false;

    // Action should not be done again
    return true;
}


