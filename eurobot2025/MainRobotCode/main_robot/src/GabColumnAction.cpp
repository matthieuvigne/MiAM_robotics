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

    // Reach the grab position
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = COLLECT_ZONE_COORDS[zoneId_].forward(forwardAmount);
    targetPosition.theta = startPosition_.theta;
    double wpt_margin = !isStartMotionBackward_ ? -20 : 20;
    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-forwardAmount/2.));
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

    // Grab, check and retry if required
    bool success = false;
    int num_attempts = 1;
    int constexpr max_attempts = 2;
    do {
      success = servoManager_->grab(front);
      if(!success)
      {
        // Prepare for retry
        std::cout << "GRAB FAILURE" << std::endl;
        if(front)
          servoManager_->frontClawOpen();
        else
          servoManager_->backClawOpen();
        robot_->getMotionController()->goStraight(front?30:-30, 0.5);
        robot_->getMotionController()->waitForTrajectoryFinished();
      } else {
        std::cout << "GRAB SUCCESS" << std::endl;
      }
      num_attempts += 1;
    } while(!success || num_attempts<=max_attempts);
    if(!success) return false;

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
    robot_->getMotionController()->goStraight(-forwardAmount*1.5, 1.0);
    // Action should not be done again
    return true;
}


