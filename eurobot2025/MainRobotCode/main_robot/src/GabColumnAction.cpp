#include "main_robot/GrabColumnAction.h"

// Distance from robot center to claw center
#define ROBOT_GRAB_OFFSET 180 - 20
#define MARGIN 200

#define BACK_EXTRA_OFFSET 48

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
    RobotPosition const frontApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(-(ROBOT_GRAB_OFFSET + MARGIN));
    RobotPosition backApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(ROBOT_GRAB_OFFSET + MARGIN);
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

    // Always grab front first
    isStartMotionBackward_ = robot_->gameState_.isFrontClawFull;

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

    double forwardAmount = (isStartMotionBackward_ ? -MARGIN - BACK_EXTRA_OFFSET : MARGIN);
    if (front)
        servoManager_->frontClawOpen();

    if (!robot_->getMotionController()->goStraight(forwardAmount, 0.75))
        return true; // Don't try again, other robot is already here.

    servoManager_->grab(front);
    robot_->gameState_.isCollectZoneFull[zoneId_] = false;
    if (isStartMotionBackward_)
    {
        robot_->gameState_.isBackClawFull = true;
    }
    else
    {
        robot_->gameState_.isFrontClawFull = true;
    }
    robot_->getMotionController()->goStraight(-forwardAmount, 0.75);
    // Action should not be done again
    return true;
}


