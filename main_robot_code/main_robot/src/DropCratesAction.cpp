#include "main_robot/DropCratesAction.h"

#define DROP_DISTANCE 170


void DropCratesAction::updateStartCondition()
{
    // TODO do not drop if zone is full
    if (robot_->getGameState()->isPantryZoneUsed[zoneId_])
    {
        priority_ = -1;
    }
    else if (robot_->getGameState()->isRobotFull && robot_->getGameState()->isClawFull)
    {
        priority_ = 10;
        if (zoneId_ == 1)
            priority_ += 2;
        if (zoneId_ == 0)
            priority_ += 5;
    }
    else if (!robot_->getGameState()->isRobotFull && !robot_->getGameState()->isClawFull)
    {
        priority_ = -1;
    }
    else if (robot_->getMatchTime() > 60.0)
    {
        priority_ = 6;
    }

    startPosition_ = PANTRY_ZONE_COORDS[zoneId_];

    // Select the most suitable dropping zones
    // TODO maybe write this better
    switch (zoneId_)
    {
        case 0:
            startPosition_.x += DROP_DISTANCE;
            startPosition_.theta = M_PI;
            break;
        case 2:
            startPosition_.y += DROP_DISTANCE;
            startPosition_.theta = -M_PI_2;
            break;
        case 3:
            startPosition_.y -= DROP_DISTANCE;
            startPosition_.theta = M_PI_2;
            break;
        case 5:
            startPosition_.y += DROP_DISTANCE;
            startPosition_.theta = -M_PI_2;
            break;
        case 6:
            startPosition_.y -= DROP_DISTANCE;
            startPosition_.theta = M_PI_2;
            break;
        case 8:
            startPosition_.y += DROP_DISTANCE;
            startPosition_.theta = -M_PI_2;
            break;
        case 9:
            startPosition_.x -= DROP_DISTANCE;
            startPosition_.theta = 0;
            break;
        default:
            // For the other pantries, choose a closest point around it
            bool feasible = false;
            double currentNorm = 0;
            RobotPosition currentPosition(robot_->getMotionController()->getCurrentPosition());
            for (int xindex=-1; xindex<2; xindex=xindex+2)
            {
                for (int yindex=-1; yindex<2; yindex=yindex+2)
                {
                    robot_->logger_ << "[DropCratesAction] xindex " << xindex << " yindex " << yindex << std::endl;
                    RobotPosition newPosition(startPosition_);
                    newPosition.x += xindex * DROP_DISTANCE;
                    newPosition.y += yindex * DROP_DISTANCE;
                    if (!feasible || ((currentPosition - newPosition).norm() < currentNorm))
                    {
                        startPosition_.x = newPosition.x;
                        startPosition_.y = newPosition.y;
                        feasible=true;
                    }
                }
            }
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

    bool dropped_something = false;
    if (robot_->getGameState()->isClawFull)
    {
        // TODO drop claw
        robot_->getGameState()->isClawFull = false;
        dropped_something = true;
    }
    else if (robot_->getGameState()->isRobotFull)
    {
        // TODO drop inside
        robot_->getGameState()->isRobotFull = false;
        dropped_something = true;
    }
    if (dropped_something)
    {
        robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;
        robot_->updateScore(4, "dropped something");

    }

    return true;
}


