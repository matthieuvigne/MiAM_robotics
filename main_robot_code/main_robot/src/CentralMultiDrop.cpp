#include "main_robot/CentralMultiDrop.h"


void CentralMultiDrop::updateStartCondition()
{
    priority_ = -1;

    if (robot_->getMatchTime() > 60 &&
        (robot_->getGameState()->isClawFull) &&
        (!robot_->getGameState()->isPantryZoneUsed[4] && !robot_->getGameState()->isPantryZoneUsed[7]))
    {
        priority_ = 20;
    }

    startPosition_ = PANTRY_ZONE_COORDS[4];
    startPosition_.x -= 200;
    startPosition_.theta = 0;
}


void CentralMultiDrop::actionStartTrigger()
{
    // Empty on purpose
}

bool CentralMultiDrop::performAction()
{
    robot_->logger_ << "[CentralMultiDrop] Starting action " << std::endl;

    RobotPosition targetPosition(2050, 800, 0);
    bool success = robot_->getMotionController()->goToStraightLine(targetPosition);
    if (!success)
        return true;


    success = robot_->getMotionController()->goStraight(-200);
    if (!success)
        return true;
    if (robot_->getGameState()->isClawFull)
    {
        if (robot_->getGameState()->isBedFull)
            servoManager_->dropCrates();
        else
            servoManager_->halfDropCrates();
        robot_->getGameState()->isPantryZoneUsed[7] = true;
    }

    success = robot_->getMotionController()->goStraight(-500);
    if (!success)
        return true;
    if (robot_->getGameState()->isBedFull)
        servoManager_->emptyBed();
    else
        servoManager_->dropCrates();
    robot_->getGameState()->isPantryZoneUsed[4] = true;
    robot_->getGameState()->isCollectZoneFull[4] = false;

    robot_->getMotionController()->goStraight(-150);
    return true;
}


