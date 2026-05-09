#include "main_robot/DropCratesAction.h"

#define MARGIN 80

void DropCratesAction::updateStartCondition()
{
    double const OFFSET = FRONT_CLAW_XOFFSET + 50.0;
    // TODO do not drop if zone is full
    if (robot_->getGameState()->isPantryZoneUsed[zoneId_])
    {
        priority_ = -1;
    }
    else if (!robot_->getGameState()->isBedFull && !robot_->getGameState()->isClawFull)
    {
        priority_ = -1;
    }
    else
    {
        priority_ = 10;
        if (zoneId_ == 1)
            priority_ += 2;
        if (zoneId_ == 2)
            priority_ += 1;
        if (zoneId_ == 0)
            priority_ += 5;
    }

    startPosition_ = PANTRY_ZONE_COORDS[zoneId_];

    // Select the most suitable dropping zones
    // TODO maybe write this better
    switch (zoneId_)
    {
        case 0:
            startPosition_.x += OFFSET;
            startPosition_.theta = M_PI;
            break;
        case 2:
            startPosition_.y += OFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 3:
            startPosition_.y -= OFFSET;
            startPosition_.theta = M_PI_2;
            break;
        case 5:
            startPosition_.y += OFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 6:
            startPosition_.y -= OFFSET;
            startPosition_.theta = M_PI_2;
            break;
        case 8:
            startPosition_.y += OFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 9:
            startPosition_.x -= OFFSET;
            startPosition_.theta = 0;
            break;
        default:
            // For the other pantries, choose a closest point around it
            double currentNorm = 100000;
            RobotPosition const currentPosition = robot_->getMotionController()->getCurrentPosition();
            RobotPosition newStart = startPosition_;

            int constexpr xindex[4] = {-1, 0, 1, 0};
            int constexpr yindex[4] = {0, -1, 0, 1};
            double constexpr thetaindex[4] = {0, M_PI_2, M_PI, -M_PI_2};
            for (int j = 0; j < 4; j++)
            {
                RobotPosition newPosition(startPosition_);
                newPosition.x += xindex[j] * OFFSET;
                newPosition.y += yindex[j] * OFFSET;
                newPosition.theta = thetaindex[j];
                double dist = (currentPosition - newPosition).norm();
                if (dist < currentNorm)
                {
                    currentNorm = dist;
                    newStart = newPosition;
                }
            }
            startPosition_ = newStart;
            break;
    }

}


void DropCratesAction::actionStartTrigger()
{
    // Empty on purpose
}

bool DropCratesAction::performAction()
{
    robot_->logger_ << "[DropCratesAction] Starting action " << zoneId_ << std::endl;

    if (robot_->getGameState()->isBedFull && robot_->getGameState()->isClawFull)
    {
        double MOVE_DIST = 70;
        double ANGLE = 0.4;
        // Move forward in arc circle.
        RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
        currentPosition.theta += ANGLE;
        RobotPosition targetPosition = currentPosition.forward(MOVE_DIST);
        robot_->getMotionController()->goToStraightLine(targetPosition);

        servoManager_->emptyBed();

        robot_->getMotionController()->goStraight(-MOVE_DIST);

        currentPosition = robot_->getMotionController()->getCurrentPosition();
        currentPosition.theta -= 2.0 * ANGLE;
        targetPosition = currentPosition.forward(MOVE_DIST);
        robot_->getMotionController()->goToStraightLine(targetPosition);
        servoManager_->moveArm(ArmPosition::GRAB);
        servoManager_->releaseSuction();
        robot_->wait(0.25);
        servoManager_->moveArm(ArmPosition::RAISE);
        robot_->getGameState()->isClawFull = false;

        robot_->getMotionController()->goStraight(-MARGIN);
    }
    else
    {
        robot_->getMotionController()->goStraight(50);
        if (robot_->getGameState()->isBedFull)
            servoManager_->emptyBed();
        else
            servoManager_->dropCrates();
        robot_->getMotionController()->goStraight(-MARGIN);
    }

    robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;
    return true;
}


