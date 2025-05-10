#include "main_robot/GrabColumnAction.h"

#define MARGIN 200

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


    //startPosition_    
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = COLLECT_ZONE_COORDS[zoneId_].forward(forwardAmount);
    targetPosition.theta = startPosition_.theta;
    double wpt_margin = !isStartMotionBackward_ ? -20 : 20;
    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-forwardAmount/2.));
    //~ positions.push_back(targetPosition);
    positions.push_back(targetPosition.forward(-wpt_margin));
    miam::trajectory::flags flag = front
      ? miam::trajectory::flags::DEFAULT
      : miam::trajectory::flags::BACKWARD;
    flag = static_cast<tf>(flag | miam::trajectory::flags::IGNORE_END_ANGLE);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                      robot_->getMotionController()->robotParams_.getTrajConf(),
                      positions,
                      100.0,
                      0.3,    // Transition velocity
                      flag
                  );
    robot_->getMotionController()->setTrajectoryToFollow(traj);
    robot_->getMotionController()->waitForTrajectoryFinished();

    servoManager_->grab(front);

    robot_->getGameState()->isCollectZoneFull[zoneId_] = false;
    if (isStartMotionBackward_)
    {
        robot_->getGameState()->isBackClawFull = true;
    }
    else
    {
        robot_->getGameState()->isFrontClawFull = true;
    }
    robot_->getMotionController()->goStraight(-forwardAmount*1.5, 1.0);
    // Action should not be done again
    return true;
}


