#include "main_robot/DropCratesAction.h"

#define MARGIN 100

void DropCratesAction::updateStartCondition()
{
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
        if (zoneId_ == 0)
            priority_ += 5;
    }

    startPosition_ = PANTRY_ZONE_COORDS[zoneId_];

    // Select the most suitable dropping zones
    // TODO maybe write this better
    switch (zoneId_)
    {
        case 0:
            startPosition_.x += FRONT_CLAW_XOFFSET;
            startPosition_.theta = M_PI;
            break;
        case 2:
            startPosition_.y += FRONT_CLAW_XOFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 3:
            startPosition_.y -= FRONT_CLAW_XOFFSET;
            startPosition_.theta = M_PI_2;
            break;
        case 5:
            startPosition_.y += FRONT_CLAW_XOFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 6:
            startPosition_.y -= FRONT_CLAW_XOFFSET;
            startPosition_.theta = M_PI_2;
            break;
        case 8:
            startPosition_.y += FRONT_CLAW_XOFFSET;
            startPosition_.theta = -M_PI_2;
            break;
        case 9:
            startPosition_.x -= FRONT_CLAW_XOFFSET;
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
                newPosition.x += xindex[j] * FRONT_CLAW_XOFFSET;
                newPosition.y += yindex[j] * FRONT_CLAW_XOFFSET;
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

    servoManager_->dropCrates();
    robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;

    // Go back from the drop zone.
    robot_->getMotionController()->goStraight(-MARGIN);
    return true;
}


