#include "main_robot/PickupPlantsAction.h"


// Radius of the circle for action start.
double const RADIUS = 200;

void PickupPlantsAction::updateStartCondition()
{
    // Start depends on robot current pose.
    RobotPosition currentPose = robot_->getMotionController()->getCurrentPosition();

    startPosition_ = PLANT_COLLECT_COORD[zoneId_];

    double const angle = std::atan2(startPosition_.y - currentPose.y, startPosition_.x - currentPose.x);
    startPosition_.x -= RADIUS * std::cos(angle);
    startPosition_.y -= RADIUS * std::sin(angle);
    startPosition_.theta = angle;
    priority_ = 1;

    // Action if feasible only if at least one claw has space
    if (robot_->gameState_.isClawAvailable(true) || robot_->gameState_.isClawAvailable(false))
        priority_ = 20;
}


void PickupPlantsAction::actionStartTrigger()
{
        robot_->logger_ << "Computing action " << robot_->gameState_.isClawAvailable(true) << std::endl;
    // Move the turret in advance
    if (robot_->gameState_.isClawAvailable(true))
        servoManager_->moveTurret(0);
    else
        servoManager_->moveTurret(M_PI);
}

bool PickupPlantsAction::performAction()
{
    servoManager_->waitForTurret();

    bool isFront = std::abs(servoManager_->getTurretPosition()) < 0.1;
        robot_->logger_ << "Is front" << isFront << std::endl;

    // Grab three plants
    robot_->getMotionController()->goToStraightLine(PLANT_COLLECT_COORD[zoneId_], 0.5);

    servoManager_->closeClaws(isFront);

    // See what has been grabbed
    robot_->wait(0.3);
    servoManager_->updateClawContent(isFront, robot_->gameState_);
    robot_->gameState_.nPlantsPerZone[zoneId_] -= 3;

    robot_->getMotionController()->goStraight(-100, 0.5);

    // Let's see if we need to go for a second grab round.
    bool shouldRetry = false;
    if (robot_->gameState_.isClawAvailable(isFront))
    {
        // Retry with same claw
        shouldRetry = true;
    }
    else if (robot_->gameState_.isClawAvailable(!isFront))
    {
        // Retry with other claw
        shouldRetry = true;
        isFront = !isFront;
        servoManager_->moveTurret(isFront ? 0 :M_PI);
        servoManager_->waitForTurret();
    }

    if (shouldRetry)
    {
        robot_->getMotionController()->goStraight(200, 0.5);

        servoManager_->closeClaws(isFront);
        robot_->logger_ << "Is front" << isFront << std::endl;

        // See what has been grabbed
        robot_->wait(0.3);
        servoManager_->updateClawContent(isFront, robot_->gameState_);
        robot_->gameState_.nPlantsPerZone[zoneId_] -= 3;
    }

    // Action should not be done again
    return true;
}


