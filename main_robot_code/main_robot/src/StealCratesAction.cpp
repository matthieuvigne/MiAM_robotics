#include "main_robot/StealCratesAction.h"

#define MARGIN 80

void StealCratesAction::updateStartCondition()
{
    if (robot_->getGameState()->isBedFull ||
        robot_->getGameState()->isClawFull ||
        robot_->getMatchTime() < 70)
    {
        priority_ = -1;
    }
    else
    {
        // Steal our side of the field first
        if (zoneId_ == 5)
            priority_ = 4;
        else if (zoneId_ == 8)
            priority_ = 3;
        else
            priority_ = 2;
    }
    ignoreFinalRotation_ = true;
    isStartMotionBackward_ = false;

    double const xoffset = FRONT_CLAW_XOFFSET + MARGIN;

    startPosition_ = PANTRY_ZONE_COORDS[zoneId_];

    switch (zoneId_)
    {
        case 0:
            startPosition_.theta = M_PI;
            break;
        case 2:
            startPosition_.theta = -M_PI_2;
            break;
        case 3:
            startPosition_.theta = M_PI_2;
            break;
        case 5:
            startPosition_.theta = -M_PI_2;
            break;
        case 6:
            startPosition_.theta = M_PI_2;
            break;
        case 8:
            startPosition_.theta = -M_PI_2;
            break;
        case 9:
            startPosition_.theta = 0;
            break;
        default:
            startPosition_.theta = M_PI_2;
    }

    startPosition_ = startPosition_.forward(-xoffset);
}


void StealCratesAction::actionStartTrigger()
{
    std::thread hide(&ServoManager::hideArm, servoManager_);
    hide.detach();
}

bool StealCratesAction::performAction()
{
    robot_->logger_ << "[StealCratesAction] Starting action " << zoneId_ << std::endl;

    // Orient robot to face the crates
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    robot_->getMotionController()->pointTurn(startPosition_.theta - currentPosition.theta);
    currentPosition = robot_->getMotionController()->getCurrentPosition();

    // Take picture, waiting for stabilisation.
    robot_->wait(0.25);
    CameraResult res = servoManager_->cameraDetectCrates(false);
    if (!res.cratesPresent)
        return true;

    RobotPosition targetPosition = currentPosition;
    targetPosition.theta = startPosition_.theta;

    // Unhide arm
    std::thread thread(&ServoManager::unhideArm, servoManager_);
    thread.detach();

    // Adjust position
    double const yTranslate = (robot_->isPlayingRightSide() ? -1.0 : 1.0) * std::clamp(res.lateralOffset, -100.0, 100.0);
    double const expectedYDiff = (startPosition_ - currentPosition).rotate(-startPosition_.theta).y;
    robot_->logger_ << "[StealCratesAction] Camera: shifting pose laterally by " << yTranslate << ", difference:" << yTranslate - expectedYDiff << std::endl;

    targetPosition = targetPosition.relativeTranslate(MARGIN + 5, yTranslate);

    robot_->logger_ << "targetPosition" << targetPosition << std::endl;
    robot_->logger_ << "start induced target" << startPosition_.forward(MARGIN + 5) << std::endl;

    robot_->getMotionController()->goToStraightLine(targetPosition.forward(-MARGIN * 0.4));
    robot_->getMotionController()->goToStraightLine(targetPosition);


    int const opponentColor = (robot_->isPlayingRightSide() ? YELLOW : BLUE);

    std::vector<int> opponentTags;
    for (unsigned int i = 0; i < res.tags.size(); i++)
    {
        if (res.tags.at(i).markerId == opponentColor)
            opponentTags.push_back(i);
    }
    // Do most favorable grab if possible.
    std::vector<int> tagsToGrab;
    if (opponentTags.size() > 2)
    {
        if (opponentTags[0] == 0)
        {
            tagsToGrab.push_back(opponentTags[0]);
            tagsToGrab.push_back(opponentTags[1]);
        }
        else
        {
            tagsToGrab.push_back(opponentTags[1]);
            tagsToGrab.push_back(opponentTags[2]);
        }
    }
    else
    {
        tagsToGrab = opponentTags;
    }
    servoManager_->grabTags(res.tags, tagsToGrab);
    servoManager_->moveCratesInBed();
    servoManager_->fingerClose();
    servoManager_->bedMidUnfold();
    robot_->getMotionController()->goStraight(-250);
    servoManager_->emptyBed();
    robot_->getMotionController()->goStraight(50);

    robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;
    robot_->getMotionController()->goStraight(-MARGIN);
    // Action should not be done again
    return true;
}


