#include "main_robot/GrabColumnAction.h"

#define MARGIN 160

void GrabColumnAction::updateStartCondition()
{
    if (!robot_->getGameState()->isCollectZoneFull[zoneId_] || (robot_->getGameState()->isFrontClawFull && robot_->getGameState()->isBackClawFull))
    {
        priority_ = -1;
    }
    else
    {
        priority_ = 5;
    }

    // Always grab front first
    isStartMotionBackward_ = robot_->getGameState()->isFrontClawFull;

    double const xoffset = (isStartMotionBackward_ ? BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET) + MARGIN;

    RobotPosition const frontApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(-xoffset);
    RobotPosition backApproach =  COLLECT_ZONE_COORDS[zoneId_].forward(xoffset);
    backApproach.theta += M_PI;

    startPosition_ = frontApproach;

    // Exclude positions outside table
    //~ if (!isPositionInTable(frontApproach))
    //~ {
        //~ startPosition_ = backApproach;
    //~ }
    //~ else
    {
        RobotPosition const currentPosition = robot_->getMotionController()->getCurrentPosition();
        if ((currentPosition - backApproach).norm() < (currentPosition - frontApproach).norm())
        {
            startPosition_ = backApproach;
        }
    }
    if (isStartMotionBackward_)
    {
        startPosition_.theta += M_PI;
    }
}


void GrabColumnAction::actionStartTrigger()
{
    servoManager_->prepareGrab(!isStartMotionBackward_);
}

bool GrabColumnAction::performAction()
{
    bool const front = !isStartMotionBackward_;

    double forwardAmount = (isStartMotionBackward_ ? -BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET);
    if (front)
        servoManager_->frontClawOpen();
    else
        servoManager_->backClawOpen();

    // Reach the grab position
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = startPosition_.forward(isStartMotionBackward_?-MARGIN:MARGIN);
    //~ targetPosition.theta = startPosition_.theta;
    double wpt_margin = !isStartMotionBackward_ ? -30 : 30;

    // For those on the side of the field, push them against the border ; try only once
    int maxGrabAttempts = 2;
    if (std::fabs(COLLECT_ZONE_COORDS[zoneId_].theta - M_PI) < 0.1)
    {
        wpt_margin = !isStartMotionBackward_ ? -60 : 60;
        maxGrabAttempts = 1;
    }

    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-forwardAmount/2.));
    positions.push_back(targetPosition.forward(-wpt_margin));
    miam::trajectory::flags flag = front
      ? miam::trajectory::flags::DEFAULT
      : miam::trajectory::flags::BACKWARD;
    flag = static_cast<tf>(flag | miam::trajectory::flags::IGNORE_END_ANGLE);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                      robot_->getMotionController()->getCurrentTrajectoryParameters(),
                      positions,
                      200.0,
                      0.3,    // Transition velocity
                      flag
                  );
    robot_->getMotionController()->setTrajectoryToFollow(traj);
    robot_->getMotionController()->waitForTrajectoryFinished();

    // Grab, check and retry if required
    bool success = false;
    int num_attempts = 1;
    while(!success && num_attempts<=maxGrabAttempts)
    {
        success = servoManager_->grab(front);
        if(!success)
        {
            // Prepare for retry
            robot_->logger_ << "[GrabColumnAction] Grab failure, retrying." << std::endl;
            if(front)
            {
                servoManager_->frontClawOpen();
                servoManager_->frontRightClaw_.openClaw();
                servoManager_->frontLeftClaw_.openClaw();
            }
            else
                servoManager_->backClawOpen();
            robot_->wait(0.3);
            robot_->getMotionController()->goStraight(front? 60:-60);
            robot_->getMotionController()->waitForTrajectoryFinished();
        }
        num_attempts += 1;
    }

    if(!success)
    {
        // We couldn't grab anything so we assume the zone is empty now.
        robot_->getGameState()->isCollectZoneFull[zoneId_] = false;
        return true;
    }

    // Update the game state
    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;
    if (isStartMotionBackward_)
    {
        robot_->getGameState()->isBackClawFull = true;
    }
    else
    {
        robot_->getGameState()->isFrontClawFull = true;
    }

    // Go back from the collect zone
    robot_->getMotionController()->goStraight(-forwardAmount*1.2, 1.0);
    // Action should not be done again
    return true;
}


