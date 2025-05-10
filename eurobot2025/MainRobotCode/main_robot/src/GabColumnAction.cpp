#include "main_robot/GrabColumnAction.h"

#define MARGIN 200

void GrabColumnAction::updateStartCondition()
{
    if (!robot_->getGameState()->isCollectZoneFull[zoneId_] || (robot_->getGameState()->isFrontClawFull && robot_->getGameState()->isBackClawFull))
    {
        priority_ = -1;
    }
    else
    {
        priority_ = 5;
    }

    // Always grab front first
    isStartMotionBackward_ = robot_->getGameState()->isFrontClawFull;

    double const xoffset = (isStartMotionBackward_ ? BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET) + MARGIN;

    RobotPosition const frontApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(-xoffset);
    RobotPosition backApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(xoffset);
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
    if (isStartMotionBackward_)
    {
        startPosition_.theta += M_PI;
    }
}


void GrabColumnAction::actionStartTrigger()
{
    servoManager_->prepareGrab(!isStartMotionBackward_);
}

bool GrabColumnAction::performAction()
{
    bool const front = !isStartMotionBackward_;

    double forwardAmount = (isStartMotionBackward_ ? -MARGIN : MARGIN);
    if (front)
        servoManager_->frontClawOpen();
    else
        servoManager_->backClawOpen();


    if (!robot_->getMotionController()->goStraight(forwardAmount, 0.75))
        return true; // Don't try again, other robot is already here.

    servoManager_->grab(front);

    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;
    if (isStartMotionBackward_)
    {
        robot_->getGameState()->isBackClawFull = true;
    }
    else
    {
        robot_->getGameState()->isFrontClawFull = true;
    }
    robot_->getMotionController()->goStraight(-forwardAmount, 0.75);
    // Action should not be done again
    return true;
}


