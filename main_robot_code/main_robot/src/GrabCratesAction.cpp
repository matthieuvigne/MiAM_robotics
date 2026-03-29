#include "main_robot/GrabCratesAction.h"

#define MARGIN 160

void GrabCratesAction::updateStartCondition()
{
    if (!robot_->getGameState()->isCollectZoneFull[zoneId_] || (robot_->getGameState()->isRobotFull && robot_->getGameState()->isClawFull))
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
    servoManager_->hideArm();
}

bool GrabCratesAction::performAction()
{
    robot_->logger_ << "[GrabCratesAction] Starting action " << zoneId_ << std::endl;


    // Reach the grab position
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = startPosition_.forward(MARGIN);

    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-MARGIN/2.));
    positions.push_back(targetPosition);

    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
        robot_->getMotionController()->getCurrentTrajectoryParameters(),
        positions,
        MARGIN / 4.0,
        0.3
    );
    robot_->getMotionController()->setTrajectoryToFollow(traj);
    robot_->getMotionController()->waitForTrajectoryFinished();

    servoManager_->grabCrates();

    robot_->getGameState()->isRobotFull = true;
    robot_->getGameState()->isClawFull = true;
    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;

    // Go back from the collect zone.
    robot_->getMotionController()->goStraight(-MARGIN);
    // Action should not be done again
    return true;
}


