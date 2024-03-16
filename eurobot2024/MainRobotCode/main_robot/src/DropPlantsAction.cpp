#include "main_robot/DropPlantsAction.h"



const miam::RobotPosition PLANT_DROP_COORD[3] =
{
    // Jardiniere
    miam::RobotPosition(200, 200, 0),
    miam::RobotPosition(200, 1800, 0),
    miam::RobotPosition(2700, 1500, 0)
};

#define CHASSIS_MARGIN 150

const miam::RobotPosition JARDINIERE_COORD[3] =
{
    // Jardiniere
    miam::RobotPosition(CHASSIS_MARGIN, 1400, M_PI),
    miam::RobotPosition(3000 - CHASSIS_MARGIN, 600, 0),
    miam::RobotPosition(760, 2000 - CHASSIS_MARGIN, M_PI_2)
};

void DropPlantsAction::updateStartCondition()
{
    startPosition_ = PLANT_DROP_COORD[zoneId_];

    // Action is only possible if there are plants present.
    if (robot_->gameState_.nPlantsInRobot() <= 0)
    {
        priority_ = -1;
    }
    else if (robot_->gameState_.nPlantsInRobot() > 4)
    {
        priority_ = 2;
    }
    else
    {
        priority_ = 0;
    }
}


void DropPlantsAction::actionStartTrigger()
{
    // TODO: we should move turret in advance
}

bool DropPlantsAction::performAction()
{
    // RobotPosition targetPosition = PLANT_DROP_COORD[zoneId_];
    // targetPosition.x = 140;

    // robot_->getMotionController()->goToStraightLine(targetPosition);

    servoManager_->setClawPosition(ClawSide::FRONT, ClawPosition::LOW_POSITION);
    robot_->wait(0.5);
    servoManager_->openClaws(true);
    robot_->gameState_.nPlantsCollected[zoneId_] += robot_->gameState_.nPlantsInRobot();

    for (int i = 0; i < 6; i++)
        robot_->gameState_.robotClawContent[i] = ClawContent::EMPTY;

    robot_->getMotionController()->goStraight(-100);

    // Action should not be done again if there are more than 3 plants.
    return robot_->gameState_.nPlantsCollected[zoneId_] > 3;
}


void DropPlantsToJarnidiereAction::updateStartCondition()
{
    RobotPosition offset = RobotPosition(-250, 0, 0).rotate(JARDINIERE_COORD[zoneId_].theta);
    offset.theta = 0;
    startPosition_ = JARDINIERE_COORD[zoneId_] + offset;

    // Action is only possible if there are plants present,
    // no plants in this jardiniere, and if the pots were cleared
    bool isPossible = robot_->gameState_.nPlantsInRobot() > 0 && robot_->gameState_.nPlantsCollected[zoneId_ + 3] == 0;
    // TODO: clear pots
    // if (zoneId_ < 2)
    //     isPossible &= robot_->gameState_.nPotsInPile[zoneId_] == 0;

    if (!isPossible)
    {
        priority_ = -1;
    }
    else if (robot_->gameState_.nPlantsInRobot() > 4)
    {
        priority_ = 2;
    }
    else
    {
        priority_ = 0;
    }
}


void DropPlantsToJarnidiereAction::actionStartTrigger()
{
    isDroppingFront_ = robot_->gameState_.isFrontClawMostFull();
    servoManager_->moveTurret(isDroppingFront_ ? 0 : M_PI);
}

bool DropPlantsToJarnidiereAction::performAction()
{
    double const MARGIN = 100;
    RobotPosition target = JARDINIERE_COORD[zoneId_] +
        RobotPosition(-MARGIN, 0, 0).rotate(JARDINIERE_COORD[zoneId_].theta);
    target.theta -= JARDINIERE_COORD[zoneId_].theta;

    if (!robot_->getMotionController()->goToStraightLine(target, 1.0, false, true))
        return false;
    if (!robot_->getMotionController()->goStraight(MARGIN + 10, 0.5))
        return false;

    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::MEDIUM_POSITION);
    robot_->wait(0.4);
    servoManager_->openClaws(isDroppingFront_);
    robot_->gameState_.nPlantsCollected[3 + zoneId_] += robot_->gameState_.clearClawContent(isDroppingFront_);

    robot_->wait(0.3);
    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);
    robot_->getMotionController()->goStraight(-MARGIN);

    // Action should never be done again
    return true;
}


