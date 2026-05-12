#include "main_robot/CursorAction.h"

#define LATERAL_DISTANCE 205

#define ARM_OFFSET 100


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
        // Action with most priority when possible.
        priority_ = 100;
    }

    startPosition_.x = (robot_->isPlayingRightSide() ? 210: 450) + ARM_OFFSET;
    startPosition_.y = LATERAL_DISTANCE;
    startPosition_.theta = (robot_->isPlayingRightSide() ? 0 : M_PI);
    isStartMotionBackward_ = true;
}


void CursorAction::actionStartTrigger()
{
    if (robot_->isPlayingRightSide())
        servoManager_->moveArm(ArmPosition::RAISE);
    else
        servoManager_->moveArm(ArmPosition::DO_CURSOR);
}

bool CursorAction::performAction()
{
    robot_->logger_ << "[CursorAction] Starting action " << std::endl;

    // Leave time for VLX to settle
    robot_->wait(0.1);
    vlx_reset();
    double const angle = (robot_->isPlayingRightSide() ? 0: M_PI);

    // Go in front of cursor
    if (!robot_->isPlayingRightSide())
    {
        RobotPosition const targetPosition(210 + ARM_OFFSET, LATERAL_DISTANCE, angle);
        robot_->getMotionController()->goToStraightLine(targetPosition);
        vlx_reset();
    }

    servoManager_->cursorUnfold();
    robot_->wait(0.1);
    tf const flags = static_cast<tf>((robot_->isPlayingRightSide() ? tf::DEFAULT : tf::BACKWARD) | tf::NO_WAIT_FOR_END);
    // // Go in front of cursor
    // {
    //     RobotPosition const targetPosition(410 + ARM_OFFSET, LATERAL_DISTANCE, angle);
    //     robot_->getMotionController()->goToStraightLine(targetPosition, 1, flags);
    // }

    // vlx_reset();
    {
        RobotPosition const targetPosition(745 + ARM_OFFSET, LATERAL_DISTANCE, angle);
        robot_->getMotionController()->goToStraightLine(targetPosition, 1, flags);
        robot_->wait(0.75);
        vlx_reset();
        robot_->getMotionController()->waitForTrajectoryFinished();
    }
    servoManager_->cursorFold();
    servoManager_->moveArm(ArmPosition::RAISE);
    robot_->getMotionController()->goStraight(robot_->isPlayingRightSide() ? - 100 : 100);

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
