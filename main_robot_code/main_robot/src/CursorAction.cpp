#include "main_robot/CursorAction.h"

#define MARGIN 160

#define LATERAL_DISTANCE 250

void CursorAction::updateStartCondition()
{

    if (robot_->getMatchTime() < 10.0)
    {
        priority_ = 1;
    }
    else
    {
        priority_ = 8;
    }

    startPosition_.x = 210;
    startPosition_.y = LATERAL_DISTANCE;
    startPosition_.theta = (robot_->isPlayingRightSide() ? 0 : M_PI);
}


void CursorAction::actionStartTrigger()
{
}

bool CursorAction::performAction()
{
    robot_->logger_ << "[CursorAction] Starting action " << std::endl;

    tf flags = (robot_->isPlayingRightSide() ? tf::DEFAULT : tf::BACKWARD);

    double const angle = (robot_->isPlayingRightSide() ? 0: M_PI);
    RobotPosition targetPosition(700, LATERAL_DISTANCE, angle);

    // Reset pose using VLX
    // Read VLX
    double const vlxY = robot_->getMeasurements()->vlxDistance;
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    if (std::abs(currentPosition.y - vlxY) < 40.0)
    {
        robot_->logger_ << "[Strategy] Resetting robot y position: " << vlxY << " instead of " << currentPosition.y << std::endl;
        currentPosition.y = vlxY;
        robot_->getMotionController()->resetPosition(currentPosition, false, true, false);
    }
    servoManager_->cursorUnfold();
    robot_->getMotionController()->goToStraightLine(targetPosition, 1, flags);
    servoManager_->cursorFold();

    // Action should not be done again
    return true;
}


