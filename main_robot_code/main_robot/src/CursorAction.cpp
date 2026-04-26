#include "main_robot/CursorAction.h"

#define LATERAL_DISTANCE 205

void CursorAction::updateStartCondition()
{
    // Don't do this action until collect zone 1 is empty
    if (robot_->getGameState()->isCollectZoneFull[1])
    {
        priority_ = 1;
        // priority_ = -1;
    }
    else
    {
        // More priority than grab, less than drop.
        priority_ = 8;
    }

    startPosition_.x = 500;
    startPosition_.y = LATERAL_DISTANCE;
    startPosition_.theta = (robot_->isPlayingRightSide() ? 0 : M_PI);
}


void CursorAction::actionStartTrigger()
{
    servoManager_->hideArm();
}

bool CursorAction::performAction()
{
    robot_->logger_ << "[CursorAction] Starting action " << std::endl;

    // Leave time for VLX to settle
    robot_->wait(0.2);
    vlx_reset();
    double const angle = (robot_->isPlayingRightSide() ? 0: M_PI);

    // Go in front of cursor
    {
        tf const flags = (robot_->isPlayingRightSide() ? tf::BACKWARD : tf::DEFAULT);
        RobotPosition const targetPosition(220, LATERAL_DISTANCE, angle);
        robot_->getMotionController()->goToStraightLine(targetPosition, 1, flags);
    }

    servoManager_->cursorUnfold();
    robot_->wait(0.8);
    vlx_reset();
    {
        tf const flags = (robot_->isPlayingRightSide() ? tf::DEFAULT : tf::BACKWARD);
        RobotPosition const targetPosition(740, LATERAL_DISTANCE, angle);
        robot_->getMotionController()->goToStraightLine(targetPosition, 1, flags);
    }
    servoManager_->cursorFold();

    // Action should not be done again
    return true;
}



void CursorAction::vlx_reset()
{
    double const vlxY = robot_->getMeasurements()->vlxDistance;
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    if (std::abs(currentPosition.y - vlxY) < 40.0)
    {
        robot_->logger_ << "[CursorAction] Resetting robot y position: " << vlxY << " instead of " << currentPosition.y << std::endl;
        currentPosition.y = vlxY;
        robot_->getMotionController()->resetPosition(currentPosition, false, true, false);
    }
    else
        robot_->logger_ << "[CursorAction] VLX reset failed: " << vlxY << " instead of " << currentPosition.y << std::endl;
}
