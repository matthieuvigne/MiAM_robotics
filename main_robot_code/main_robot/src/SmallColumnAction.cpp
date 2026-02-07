// #include "main_robot/SmallColumnAction.h"

// #define MARGIN 130

// #define ZONE_ID 8

// void SmallColumnAction::updateStartCondition()
// {
//     if (robot_->getGameState()->isFrontClawFull)
//     {
//         priority_ = -1;
//     }
//     else
//     {
//         priority_ = 50;
//         if (robot_->getMatchTime() > 20)
//             priority_ += 10;
//     }
//     isStartMotionBackward_ = false;
//     ignoreFinalRotation_ = true;


//     startPosition_ =  COLLECT_ZONE_COORDS[ZONE_ID].forward(FRONT_CLAW_XOFFSET + MARGIN);
//     startPosition_.theta = -M_PI_2;
// }


// void SmallColumnAction::actionStartTrigger()
// {
// }

// bool SmallColumnAction::performAction()
// {
//     robot_->logger_ << "[SmallColumnAction] Starting action." << std::endl;
//     servoManager_->prepareGrab(true);


//     // Reach the grab position
//     RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
//     RobotPosition targetPosition = startPosition_.forward(isStartMotionBackward_?-MARGIN:MARGIN);
//     //~ targetPosition.theta = startPosition_.theta;
//     std::vector<RobotPosition> positions;
//     positions.push_back(currentPosition);
//     positions.push_back(targetPosition.forward(-MARGIN / 2.0));
//     positions.push_back(targetPosition.forward(30));


//     TrajectoryConfig params = robot_->getMotionController()->getCurrentTrajectoryParameters();
//     params.maxWheelAcceleration = 300;

//     TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
//                       params,
//                       positions,
//                       70.0,
//                       0.15,    // Transition velocity
//                       miam::trajectory::flags::IGNORE_END_ANGLE
//                   );
//     robot_->getMotionController()->setTrajectoryToFollow(traj);
//     robot_->getMotionController()->waitForTrajectoryFinished();

//     // Grab, check and retry if required
//     bool success = false;
//     int num_attempts = 1;
//     int constexpr max_attempts = 2;
//     success = servoManager_->grab(true);
//     while(!success && num_attempts<=max_attempts)
//     {
//         servoManager_->frontClawOpen();
//         servoManager_->frontRightClaw_.openClaw();
//         servoManager_->frontLeftClaw_.openClaw();
//         robot_->logger_ << "[SmallColumnAction] Grab failure, retrying." << std::endl;
//         robot_->getMotionController()->goStraight(50, 0.5);
//         robot_->getMotionController()->waitForTrajectoryFinished();
//         success = servoManager_->grab(true, !(num_attempts == max_attempts - 1));
//         num_attempts += 1;
//     }

//     // Clear zone
//     robot_->getGameState()->isCollectZoneFull[ZONE_ID] = false;

//     // Abort if we didn't grab enough.
//     if(!success)
//     {
//         int nGrabbed = servoManager_->countGrab(true);
//         int err;
//         if (servoManager_->frontRightClaw_.isClawFull(err))
//             nGrabbed ++;
//         if (servoManager_->frontLeftClaw_.isClawFull(err))
//             nGrabbed ++;
//         if (nGrabbed < 2)
//             return true;
//     }

//     // Move forward and build
//     targetPosition = robot_->getMotionController()->getCurrentPosition();
//     targetPosition.x = 720;
//     targetPosition.y = FRONT_CLAW_XOFFSET + 80;
//     targetPosition.theta = -M_PI_2;
//     robot_->getMotionController()->goToStraightLine(targetPosition);

//     bool lvl2 = servoManager_->buildFrontTower();

//     // Update game state and score
//     robot_->getGameState()->isConstructionZoneUsed[2] = true;
//     if (lvl2)
//         robot_->updateScore(12, "Lvl.2 tower");
//     else
//         robot_->updateScore(4, "Lvl.1 tower");

//     robot_->getMotionController()->goStraight(-MARGIN);
//     servoManager_->clawsToMoveConfiguration(true);
//     // Action should not be done again
//     return true;
// }


