#include "main_robot/SmallColumnAction.h"

#define MARGIN 160

#define ZONE_ID 8

void SmallColumnAction::updateStartCondition()
{
    if (robot_->getGameState()->isFrontClawFull)
    {
        priority_ = -1;
    }
    else
    {
        priority_ = 50;
        if (robot_->getMatchTime() > 20)
            priority_ += 10;
    }
    isStartMotionBackward_ = false;


    startPosition_ =  COLLECT_ZONE_COORDS[ZONE_ID].forward(FRONT_CLAW_XOFFSET + MARGIN);
    startPosition_.theta = -M_PI_2;
}


void SmallColumnAction::actionStartTrigger()
{
    servoManager_->prepareGrab(true);
}

bool SmallColumnAction::performAction()
{

    servoManager_->frontClawOpen();

    // Reach the grab position
    RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
    RobotPosition targetPosition = startPosition_.forward(isStartMotionBackward_?-MARGIN:MARGIN);
    //~ targetPosition.theta = startPosition_.theta;
    std::vector<RobotPosition> positions;
    positions.push_back(currentPosition);
    positions.push_back(targetPosition.forward(-MARGIN / 2.0));
    positions.push_back(targetPosition.forward(30));

    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                      robot_->getMotionController()->robotParams_.getTrajConf(),
                      positions,
                      100.0,
                      0.3,    // Transition velocity
                      miam::trajectory::flags::IGNORE_END_ANGLE
                  );
    robot_->getMotionController()->setTrajectoryToFollow(traj);
    robot_->getMotionController()->waitForTrajectoryFinished();

    // Grab, check and retry if required
    bool success = false;
    int num_attempts = 1;
    int constexpr max_attempts = 3;
    do {
      success = servoManager_->grab(true);
      if(!success)
      {
        // Prepare for retry
        servoManager_->frontClawOpen();
        robot_->getMotionController()->goStraight(50, 0.5);
        robot_->getMotionController()->waitForTrajectoryFinished();
      }
      num_attempts += 1;
    } while(!success && num_attempts<=max_attempts);

    // Abort action
    if(!success)
        return false;

    // Move forward and build
    robot_->getMotionController()->goStraight(robot_->getMotionController()->getCurrentPosition().y - FRONT_CLAW_XOFFSET - 90);

    servoManager_->buildFrontTower();

    // Update game state and score
    robot_->getGameState()->isCollectZoneFull[ZONE_ID] = false;
    robot_->getGameState()->isConstructionZoneUsed[2] = true;
    robot_->updateScore(12, "Lvl.2 tower");

    robot_->getMotionController()->goStraight(-MARGIN);
    // Action should not be done again
    return true;
}


