#include "main_robot/BuildAction.h"

#define MARGIN 120
#define BUILD_DISTANCE 140


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

    specialReverse_ = zoneId_ == 0 && nDrop_ == 0;
    // Default to back drop on large zones, front drop on small zones
    if (largeZone_)
    {
       isStartMotionBackward_ = robot_->getGameState()->isBackClawFull;
    }
    else
        isStartMotionBackward_ = !robot_->getGameState()->isFrontClawFull;

    // We want to build at CONSTRUCTION_ZONE_COORDS + BUILD_DISTANCE * nDrop_
    // yoffset is here to position the robot so that when it moves back by MARGIN, the cans are in the right place.
    double const yoffset = (isStartMotionBackward_ ? BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET) + MARGIN;
    startPosition_ = CONSTRUCTION_ZONE_COORDS[zoneId_].forward(-yoffset - BUILD_DISTANCE * nDrop_);

    if (isStartMotionBackward_)
        startPosition_.theta += M_PI;

    // Go forward for first drop
    if (specialReverse_)
        isStartMotionBackward_ = false;
}


void BuildAction::actionStartTrigger()
{
    // Empty on purpose
}

bool BuildAction::performAction()
{
    robot_->logger_ << "[BuildAction] Starting action " << zoneId_ << std::endl;
    if (specialReverse_)
        isStartMotionBackward_ = true;
    int const sign = (isStartMotionBackward_ ? -1 : 1);

    // Move the cans in the planned spot.
    robot_->getMotionController()->goStraight(sign * MARGIN);
    if (isStartMotionBackward_)
    {
        servoManager_->dropBackCans();
        robot_->getGameState()->isBackClawFull = false;
        robot_->updateScore(4, "Lvl.1 tower");
    }
    else
    {
        bool lvl2 = servoManager_->buildFrontTower();
        robot_->getGameState()->isFrontClawFull = false;
        if (lvl2)
            robot_->updateScore(12, "Lvl.2 tower");
        else
            robot_->updateScore(4, "Lvl.1 tower");
    }
    robot_->getGameState()->isConstructionZoneUsed[zoneId_] = true;

    if (isStartMotionBackward_ && robot_->getGameState()->isFrontClawFull && (largeZone_))
    {
        // Build front tower, put it on top of the other one
        robot_->getMotionController()->goStraight(MARGIN + BUILD_DISTANCE);

        RobotPosition target = startPosition_;
        target.x = robot_->getMotionController()->getCurrentPosition().x;
        target.y = CONSTRUCTION_ZONE_COORDS[zoneId_].y +  BUILD_DISTANCE * (nDrop_ + 1) + FRONT_CLAW_XOFFSET;
        target.theta += M_PI;

        bool moveSuccesful;
        moveSuccesful = robot_->getMotionController()->goToStraightLine(target);
        if (!moveSuccesful)
        {
            robot_->logger_ << "[BuildAction] Could not perform build due to detection, aborting" << std::endl;
            nDrop_ ++;
            return nDrop_ > 2;
        }
        bool lvl2 = servoManager_->buildFrontTower();
        robot_->getGameState()->isFrontClawFull = false;
        if (lvl2)
            robot_->updateScore(12, "Lvl.2 tower");
        else
            robot_->updateScore(4, "Lvl.1 tower");

        moveSuccesful = robot_->getMotionController()->goStraight(-MARGIN);
        servoManager_->clawsToMoveConfiguration(true);

        if (moveSuccesful && lvl2)
        {
            double towerPenetration = 0;
            // Try to build a level 3 column
            robot_->getMotionController()->pointTurn(M_PI);
            robot_->getMotionController()->goStraight(-(MARGIN + BACK_DIFF_XOFFSET + towerPenetration), 0.5);
            servoManager_->backClawClose();
            servoManager_->grabBackTwoPlanks();
            robot_->wait(0.3);
            if (!servoManager_->checkGrab(false))
            {
                // If the base was not grabbed successfully, abort
                robot_->logger_ << "[BuildAction] Back grab fail, abort level 3" << std::endl;
                servoManager_->backClawOpen();
                servoManager_->releaseBackPlank();
                robot_->wait(0.4);
                robot_->getMotionController()->goStraight(MARGIN);

                servoManager_->clawsToMoveConfiguration(false);
                nDrop_ +=2;
                return nDrop_ > 2;
            }
            servoManager_->backRail_.move(0.98);
            while (servoManager_->backRail_.isMoving())
                robot_->wait(0.050);
            robot_->getMotionController()->enableDetection(false);
            robot_->getMotionController()->goStraight(-BUILD_DISTANCE + towerPenetration, 0.3);
            robot_->getMotionController()->enableDetection(true);

            servoManager_->dropBackCans(false);
            robot_->updateScore(12, "Lvl.3 tower");
            robot_->getMotionController()->goStraight(2. * MARGIN);
            servoManager_->clawsToMoveConfiguration(false);
        }
    }
    else
    {
        robot_->getMotionController()->goStraight(-sign * 200);
        servoManager_->clawsToMoveConfiguration(isStartMotionBackward_);
    }

    // Allow several drops in large zones
    if (largeZone_)
    {
        nDrop_ ++;
        return nDrop_ > 2;
    }
    return true;
}


