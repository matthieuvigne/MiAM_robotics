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
        switch(zoneId_)
        {
            case 0: priority_ = 3; break;
            case 1: priority_ = 3; break;
            case 3: priority_ = 5; break;
            case 5: priority_ = 4; break;
            case 7: priority_ = 1; break;
            default: priority_ = 2; break;
        }
    }
    ignoreFinalRotation_ = true;

    // Always grab front first
    isStartMotionBackward_ = false;

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


void GrabCratesAction::actionStartTrigger()
{
}

bool GrabCratesAction::performAction()
{
    robot_->logger_ << "[GrabCratesAction] Starting action " << zoneId_ << " " << isStartMotionBackward_ << std::endl;
    bool const front = !isStartMotionBackward_;

    // servoManager_->prepareGrab(!isStartMotionBackward_);

    double forwardAmount = (isStartMotionBackward_ ? -BACK_CLAW_XOFFSET : FRONT_CLAW_XOFFSET);
    // if (front)
    // {
    //     servoManager_->frontClawOpen();
    //     servoManager_->frontRightClaw_.rail_.move(0.05);
    //     servoManager_->frontLeftClaw_.rail_.move(0.05);
    //     while (servoManager_->frontRightClaw_.rail_.isMoving()
    //         || servoManager_->frontLeftClaw_.rail_.isMoving())
    //             robot_->wait(0.050);
    // }
    // else
    // {
    //     servoManager_->backClawOpen();
    // }

    // Reach the grab position
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = startPosition_.forward(isStartMotionBackward_?-MARGIN:MARGIN);
    //~ targetPosition.theta = startPosition_.theta;
    double wpt_margin = !isStartMotionBackward_ ? -40 : 40;

    // For those on the side of the field, push them against the border ; try only once
    int maxGrabAttempts = 2;
    if (std::fabs(std::fabs(COLLECT_ZONE_COORDS[zoneId_].theta) - M_PI_2) > 0.1)
    {
        wpt_margin = !isStartMotionBackward_ ? -35 : 35;
        maxGrabAttempts = 0;
    }
    // Don't enter opponent zone
    if (zoneId_ == 7)
        maxGrabAttempts = 1;

    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-forwardAmount/2.));
    positions.push_back(targetPosition.forward(-wpt_margin));
    miam::trajectory::flags flag = front
      ? miam::trajectory::flags::DEFAULT
      : miam::trajectory::flags::BACKWARD;
    flag = static_cast<tf>(flag | miam::trajectory::flags::IGNORE_END_ANGLE);
    TrajectoryConfig params = robot_->getMotionController()->getCurrentTrajectoryParameters();
    params.maxWheelAcceleration = 300;

    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                      params,
                      positions,
                      70.0,
                      0.15,    // Transition velocity
                      flag
                  );
    robot_->getMotionController()->setTrajectoryToFollow(traj);
    robot_->getMotionController()->waitForTrajectoryFinished();

    // Grab, check and retry if required
    bool success = false;
    int num_attempts = 1;
    // success = servoManager_->grab(front);
    while(!success && num_attempts<=maxGrabAttempts)
    {
        robot_->logger_ << "[GrabCratesAction] Grab failure, retrying." << std::endl;
        // if(front)
        // {
        //     servoManager_->frontClawOpen();
        //     servoManager_->frontRightClaw_.openClaw();
        //     servoManager_->frontLeftClaw_.openClaw();
        // }
        // else
        //     servoManager_->backClawOpen();
        robot_->wait(0.3);
        robot_->getMotionController()->goStraight(front? 60:-60);
        robot_->getMotionController()->waitForTrajectoryFinished();

        success = true; //servoManager_->grab(front, !(num_attempts == maxGrabAttempts - 1));
        num_attempts += 1;
    }

    // if robot is not full
    // TODO and match time remaining is still large,
    // TODO then store the crates
    if (!robot_->getGameState()->isRobotFull)
    {
        robot_->getGameState()->isRobotFull = true;
    }
    else
    {
        robot_->getGameState()->isClawFull = true;
    }

    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;

    // Go back from the collect zone ; move further to avoid oponnent zone
    if (zoneId_ == 1)
        robot_->getMotionController()->goStraight((isStartMotionBackward_ ? 500 : -500), 1.0);
    else
        robot_->getMotionController()->goStraight((isStartMotionBackward_ ? 1.0 : -1.0) * MARGIN, 1.0);
    // Action should not be done again
    return true;
}


