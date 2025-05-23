#include "main_robot/PickupPlantsAction.h"


// Radius of the circle for action start.
double const RADIUS = 350;

void PickupPlantsAction::updateStartCondition()
{
    // Start depends on robot current pose.
    RobotPosition currentPose = robot_->getMotionController()->getCurrentPosition();

    startPosition_ = PLANT_COLLECT_COORD[zoneId_];

    double angle = std::atan2(startPosition_.y - currentPose.y, startPosition_.x - currentPose.x);

    // For the zones on the line, only grab the plants horizontally.
    if (std::abs(startPosition_.x - 1500) > 10)
    {
        if (currentPose.x < startPosition_.x)
            angle = 0;
        else
            angle = M_PI;
    }
    startPosition_.x -= RADIUS * std::cos(angle);
    startPosition_.y -= RADIUS * std::sin(angle);
    startPosition_.theta = angle;
    priority_ = 1;

    // Action if feasible only if at least one claw has space
    if (robot_->gameState_.isClawAvailable(true) || robot_->gameState_.isClawAvailable(false))
        priority_ = 20;
        
    // Deactivate zone with ID 5 until
    if(this->zoneId_==5)
      priority_ = (robot_->getMatchTime()>50) ? priority_ : -1;
}


void PickupPlantsAction::actionStartTrigger()
{
    robot_->logger_ << "Computing action " << robot_->gameState_.isClawAvailable(true) << std::endl;
    // Move the turret in advance, but turn it only if needed
    bool const isFront = std::abs(servoManager_->getTurretPosition()) < 0.1;

    int const clawDiff = robot_->gameState_.frontToBackClawDiff();
    if (clawDiff > 0 && isFront)
        servoManager_->moveTurret(M_PI);
    else if (clawDiff < 0 && !isFront)
        servoManager_->moveTurret(0);
}

bool PickupPlantsAction::performAction()
{
    robot_->logger_ << "[PickupPlantsAction] Performing action" << std::endl;
    servoManager_->waitForTurret();

    bool isFront = std::abs(servoManager_->getTurretPosition()) < 0.1;
    robot_->logger_ << "Is front" << isFront << std::endl;

    servoManager_->openAvailableClaws(isFront, robot_->gameState_);
    bool succeeded = servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
    if (!succeeded)
    {
        // We landed on some plants: let's move back and try again.
        servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);
        robot_->wait(0.3);
        if (!robot_->getMotionController()->goStraight(-80))
            return true; // Don't try again, other robot is already here.

        servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
    }

    // Grab three plants, moving toward the center of the zone.
    RobotPosition currentPose = robot_->getMotionController()->getCurrentPosition();
    double const angle = std::atan2(PLANT_COLLECT_COORD[zoneId_].y - currentPose.y, PLANT_COLLECT_COORD[zoneId_].x - currentPose.x);

    RobotPosition targetPosition = currentPose;
    targetPosition.theta = angle;
    targetPosition.x += 160 * std::cos(angle);
    targetPosition.y += 160 * std::sin(angle);
    if (!robot_->getMotionController()->goToStraightLine(targetPosition, 0.5, tf::IGNORE_END_ANGLE))
        return true; // Don't try again, other robot is already here.

    servoManager_->closeClaws(isFront);

    // See what has been grabbed
    robot_->wait(0.15);
    robot_->gameState_.nPlantsPerZone[zoneId_] -= servoManager_->updateClawContent(isFront, robot_->gameState_);

    // Let's see if we need to go for a second grab round.
    bool shouldRetry = false;
    double forwardAmount = 120;
    if (robot_->gameState_.isClawAvailable(isFront))
    {
        // Retry with same claw
        shouldRetry = true;
    }
    else if (robot_->gameState_.isClawAvailable(!isFront))
    {
        // Retry with other claw

        // First raise the current claw
        servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);

        // Go back to avoid dropping a claw on some plants
        if (!robot_->getMotionController()->goStraight(-50, 0.5))
            return true; // Don't try again, other robot is already here.
        forwardAmount += 50;

        shouldRetry = true;
        isFront = !isFront;
        servoManager_->moveTurret(isFront ? 0 :M_PI);
        robot_->wait(0.5);
        servoManager_->waitForTurret();
        succeeded = servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
        if (!succeeded)
        {
            // We landed on some plants: let's move back and try again.
            servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);
            robot_->wait(0.3);
            robot_->getMotionController()->goStraight(-80);
            servoManager_->setClawPosition((isFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
        }
    }

    if (shouldRetry)
    {
        servoManager_->openAvailableClaws(isFront, robot_->gameState_);
        if (!robot_->getMotionController()->goStraight(forwardAmount, 0.5))
            return true; // Don't try again, other robot is already here.

        servoManager_->closeClaws(isFront);

        // See what has been grabbed
        robot_->wait(0.3);
        robot_->gameState_.nPlantsPerZone[zoneId_] -= servoManager_->updateClawContent(isFront, robot_->gameState_);
    }

    servoManager_->setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    servoManager_->setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);

    // Action should not be done again
    return true;
}


