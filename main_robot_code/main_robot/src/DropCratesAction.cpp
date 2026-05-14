#include "main_robot/DropCratesAction.h"

#define MARGIN 80

#define LATERAL_OFFSET 50.0

void DropCratesAction::updateStartCondition()
{
    double const OFFSET = FRONT_CLAW_XOFFSET + 30.0;
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
            startPosition_.y -= 80; // Space for PAMI
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

    if (std::abs(startPosition_.theta - 0) < 1e-6)
    {
        startPosition_.y += LATERAL_OFFSET - 20.0;
    }
    else if (std::abs(startPosition_.theta - M_PI) < 1e-6)
    {
        startPosition_.y -= LATERAL_OFFSET - 20.0;
    }
    else if (std::abs(startPosition_.theta - M_PI_2) < 1e-6)
    {
        startPosition_.x -= LATERAL_OFFSET - 20.0;
    }
    else if (std::abs(startPosition_.theta + M_PI_2) < 1e-6)
    {
        startPosition_.x += LATERAL_OFFSET - 20.0;
    }

}


void DropCratesAction::actionStartTrigger()
{
    if (robot_->getGameState()->isBedFull && robot_->getGameState()->isClawFull)
        servoManager_->moveRails(RailPosition::DROP);
}

bool DropCratesAction::performAction()
{
    robot_->logger_ << "[DropCratesAction] Starting action " << zoneId_ << std::endl;

    if (robot_->getGameState()->isBedFull && robot_->getGameState()->isClawFull)
    {
        servoManager_->dropCrates();
        servoManager_->moveRails(RailPosition::FORWARD);
        robot_->getMotionController()->goStraight(-MARGIN);
        // servoManager_->bedMidUnfold();

        RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
        std::vector<RobotPosition> positions;
        positions.push_back(currentPosition);
        positions.push_back(currentPosition.relativeTranslate(MARGIN / 2.0, -2 * LATERAL_OFFSET));
        positions.push_back(currentPosition.relativeTranslate(MARGIN, -2 * LATERAL_OFFSET));
        TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
            robot_->getMotionController()->getCurrentTrajectoryParameters(),
            positions,
            MARGIN / 8.0,
            0.15
        );
        robot_->getMotionController()->setTrajectoryToFollow(traj);
        robot_->getMotionController()->waitForTrajectoryFinished();

        servoManager_->emptyBed();
        robot_->getMotionController()->goStraight(-MARGIN);
    }
    else
    {
        if (robot_->getGameState()->isBedFull)
        {
            // servoManager_->bedMidUnfold();
            // robot_->wait(0.4);
            servoManager_->emptyBed();
        }
        else
        {
            if (!robot_->getGameState()->isClawHalfFull &&
                robot_->getMatchTime() > 82 && robot_->getMatchTime() < 89)
                servoManager_->halfDropCrates();
            else
                servoManager_->dropCrates();
        }
        robot_->getMotionController()->goStraight(-MARGIN);
    }

    robot_->getGameState()->isPantryZoneUsed[zoneId_] = true;
    return true;
}


