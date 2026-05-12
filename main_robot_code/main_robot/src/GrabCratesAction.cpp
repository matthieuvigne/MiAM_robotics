#include "main_robot/GrabCratesAction.h"

#define MARGIN 80

void GrabCratesAction::updateStartCondition()
{
    if (!robot_->getGameState()->isCollectZoneFull[zoneId_] || robot_->getGameState()->isBedFull || robot_->getGameState()->isClawFull)
    {
        priority_ = -1;
    }
    else
    {
        // Grab our side of the field first
        if (zoneId_ < 4)
            priority_ = 5;
        else
            priority_ = 2;
    }
    ignoreFinalRotation_ = true;
    isStartMotionBackward_ = false;

    double const xoffset = FRONT_CLAW_XOFFSET + MARGIN;

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
}


void GrabCratesAction::actionStartTrigger()
{
    std::thread hide(&ServoManager::hideArm, servoManager_);
    hide.detach();
}

bool GrabCratesAction::performAction()
{
    robot_->logger_ << "[GrabCratesAction] Starting action " << zoneId_ << std::endl;

    // Orient robot to face the crates
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    robot_->getMotionController()->pointTurn(startPosition_.theta - currentPosition.theta);
    currentPosition = robot_->getMotionController()->getCurrentPosition();

    // Take picture
    robot_->wait(0.15);
    CameraResult res = servoManager_->cameraDetectCrates();
    RobotPosition targetPosition = currentPosition;
    if (!res.cratesPresent)
        return true;

        // Unhide arm
    std::thread thread(&ServoManager::unhideArm, servoManager_);
    thread.detach();

    // Adjust position
    double const yTranslate = (robot_->isPlayingRightSide() ? -1.0 : 1.0) * std::clamp(res.lateralOffset, -100.0, 100.0);
    double const expectedYDiff = (startPosition_ - currentPosition).rotate(-startPosition_.theta).y;
    robot_->logger_ << "[GrabCratesAction] Camera: shifting pose laterally by " << yTranslate << ", difference:" << yTranslate - expectedYDiff << std::endl;

    targetPosition = targetPosition.relativeTranslate(MARGIN + 5, yTranslate);

    robot_->logger_ << "targetPosition" << targetPosition << std::endl;
    robot_->logger_ << "start induced target" << startPosition_.forward(MARGIN + 5) << std::endl;

    robot_->getMotionController()->goToStraightLine(targetPosition.forward(-MARGIN * 0.4));
    robot_->getMotionController()->goToStraightLine(targetPosition);

    servoManager_->grabCrates(res);
    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;
    // Go back from the collect zone.
    robot_->getMotionController()->goStraight(-MARGIN);
    // Action should not be done again
    return true;
}


