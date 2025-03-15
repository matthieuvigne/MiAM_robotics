#include "main_robot/GrabColumnAction.h"

void GrabColumnAction::updateStartCondition()
{
    if (!robot_->gameState_.isCollectZoneFull[zoneId_] || (robot_->gameState_.isFrontClawFull && robot_->gameState_.isBackClawFull))
    {
        priority_ = -1;
    }
    else
    {
        priority_ = 5;
    }
    RobotPosition const frontApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(-250);
    RobotPosition backApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(250);
    backApproach.theta += M_PI;

    startPosition_ = frontApproach;

    // Exclude positions outside table
    if (!isPositionInTable(frontApproach))
    {
        startPosition_ = backApproach;
    }
    else
    {
        RobotPosition const currentPosition = robot_->getMotionController()->getCurrentPosition();
        if ((currentPosition - backApproach).norm() < (currentPosition - frontApproach).norm())
        {
            startPosition_ = backApproach;
        }
    }


    isStartMotionBackward_ = robot_->gameState_.isFrontClawFull;

    if (isStartMotionBackward_)
    {
        startPosition_.theta += M_PI;
    }
}


void GrabColumnAction::actionStartTrigger()
{
    // TODO
}

bool GrabColumnAction::performAction()
{
    robot_->gameState_.isCollectZoneFull[zoneId_] = false;
    if (isStartMotionBackward_)
    {
        robot_->gameState_.isBackClawFull = true;
    }
    else
    {
        robot_->gameState_.isFrontClawFull = true;
    }
    // Action should not be done again
    return true;
}


