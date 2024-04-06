#include "main_robot/DropPlantsAction.h"

#define CHASSIS_MARGIN 140
#define POT_MARGIN 110

const miam::RobotPosition PLANT_DROP_COORD[3] =
{
    miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 612, 0),
    miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 1388, 0),
    miam::RobotPosition(3000 - (CHASSIS_MARGIN + POT_MARGIN + 100), 612, M_PI)
};

const miam::RobotPosition JARDINIERE_COORD[3] =
{
    // Jardiniere
    miam::RobotPosition(3000 - CHASSIS_MARGIN, 600, 0),
    miam::RobotPosition(CHASSIS_MARGIN, 1400, M_PI),
    miam::RobotPosition(760, 2000 - CHASSIS_MARGIN, M_PI_2)
};

const int DROP_PRIORITY_PREFERENCE[6] = {
    0, // This zone is used for the solar panels, and might be used at the end.
    4,
    1,
    1,
    2,
    5
    };

void dropPlants(RobotInterface *robot, ServoManager *servos, bool dropFront, int zoneId, bool dropGround = false)
{
    servos->waitForTurret();
    if (dropGround)
    {
        servos->setClawPosition((dropFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
        robot->wait(1.0);
    }
    servos->openClaws(dropFront);
    int nPlants = -servos->updateClawContent(dropFront, robot->gameState_);
    robot->gameState_.nPlantsCollected[zoneId] += nPlants;
    robot->updateScore(3 * nPlants);
}

void DropPlantsAction::updateStartCondition()
{
    startPosition_ = PLANT_DROP_COORD[zoneId_];
    isStartMotionBackward_ = true;

    // Action is only possible if there are plants present.
    bool isPossible = robot_->gameState_.nPlantsInRobot() > 0 && robot_->gameState_.nPlantsCollected[zoneId_] == 0;

    if (!isPossible)
    {
        priority_ = -1;
    }
    else if (robot_->gameState_.nPlantsInRobot() > 4)
    {
        priority_ = 5 + DROP_PRIORITY_PREFERENCE[zoneId_];
    }
    else
    {
        priority_ = DROP_PRIORITY_PREFERENCE[zoneId_];
    }
}


void DropPlantsAction::actionStartTrigger()
{
    isDroppingFront_ = robot_->gameState_.isFrontClawMostFull();
    // Action will be done backward.
    servoManager_->moveTurret(isDroppingFront_ ? M_PI : 0);
}

bool DropPlantsAction::performAction()
{
    // Push pots, moving backward.
    RobotPosition offset = RobotPosition(-50, 0, 0).rotate(PLANT_DROP_COORD[zoneId_].theta);
    offset.theta = 0;
    RobotPosition target = PLANT_DROP_COORD[zoneId_] + offset;

    if (!robot_->getMotionController()->goToStraightLine(target, 1.0, tf::BACKWARD))
        return false;

    servoManager_->turnOnMagnets();
    servoManager_->closeElectromagnetArms();
    robot_->wait(0.3);
    if (!robot_->getMotionController()->goStraight(-50, 0.4))
        return false;
    if (!robot_->getMotionController()->goStraight(50))
        return false;

    // Go to zone, pushing the pots
    target = robot_->getMotionController()->getCurrentPosition();
    std::vector<RobotPosition> positions;
    int yInvert = (zoneId_ == 1 ? 1 : -1);
    int xInvert = (zoneId_ == 0 ? -1 : 1);

    target.y += 200 * yInvert;
    target.x += 45 * xInvert;
    positions.push_back(target);
    target.y += 160 * yInvert;
    positions.push_back(target);

    if (!robot_->getMotionController()->goToRoundedCorners(positions, 200, 0.3, static_cast<tf>(tf::BACKWARD | tf::IGNORE_END_ANGLE)))
    {
        // In the unlikely event of a failure here: drop everything in place.
        servoManager_->openElectromagnetArms();
        servoManager_->turnOffMagnets();
        return false;
    }

    // Drop plants
    servoManager_->turnOffMagnets();
    robot_->getMotionController()->goStraight(20);
    dropPlants(robot_, servoManager_, isDroppingFront_, zoneId_);
    servoManager_->openElectromagnetArms();

    // Are there other plants to drop ?
    isDroppingFront_ = !isDroppingFront_;

    tf flag = tf::DEFAULT;
    if (robot_->gameState_.nPlantsInRobot() > 0)
    {
        robot_->getMotionController()->goStraight(150);
        servoManager_->moveTurret(isDroppingFront_ ? 0 : M_PI);
        robot_->getMotionController()->pointTurn(M_PI);
        dropPlants(robot_, servoManager_, isDroppingFront_, zoneId_, true);
        robot_->getMotionController()->goStraight(-50);
        servoManager_->setClawPosition(isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK, ClawPosition::HIGH_POSITION);
        robot_->wait(0.8);
        flag = tf::BACKWARD;
    }
    else
    {
        robot_->getMotionController()->goStraight(80);
    }

    // Finish by pusing the pots out of the way
    target = PLANT_DROP_COORD[zoneId_];
    target.y += - 300 * yInvert;

    // Action is done, pots simply weren't pushed
    if (!robot_->getMotionController()->goToStraightLine(target, 1.0, static_cast<tf>(flag | tf::IGNORE_END_ANGLE)))
        return true;

    servoManager_->setClawPosition(isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK, ClawPosition::LOW_POSITION);

    positions.clear();
    target.x += -(50 + POT_MARGIN) * xInvert;
    target.y += 200 * yInvert;
    positions.push_back(target);
    target.y += 250 * yInvert;
    positions.push_back(target);

    if (robot_->getMotionController()->goToRoundedCorners(positions, 100, 0.3, tf::IGNORE_END_ANGLE))
    {
        robot_->gameState_.nPotsInPile[zoneId_] = 0;
        robot_->getMotionController()->goStraight(-50);
    }

    servoManager_->setClawPosition(isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK, ClawPosition::HIGH_POSITION);

    return true;
}


void DropPlantsToJarnidiereAction::updateStartCondition()
{
    RobotPosition offset = RobotPosition(-250, 0, 0).rotate(JARDINIERE_COORD[zoneId_].theta);
    offset.theta = 0;
    startPosition_ = JARDINIERE_COORD[zoneId_] + offset;

    // Action is only possible if there are plants present,
    // no plants in this jardiniere, and if the pots were cleared
    bool isPossible = robot_->gameState_.nPlantsInRobot() > 0 && robot_->gameState_.nPlantsCollected[zoneId_ + 3] == 0;
    // A:ction is only possible if pots have been cleared.
    if (zoneId_ < 2)
        isPossible &= robot_->gameState_.nPotsInPile[zoneId_] == 0;

    if (!isPossible)
    {
        priority_ = -1;
    }
    else if (robot_->gameState_.nPlantsInRobot() > 4)
    {
        priority_ = 5 + DROP_PRIORITY_PREFERENCE[3 + zoneId_];
    }
    else
    {
        priority_ = DROP_PRIORITY_PREFERENCE[3 + zoneId_];
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

    if (!robot_->getMotionController()->goToStraightLine(target, 1.0))
        return false;
    if (!robot_->getMotionController()->goStraight(MARGIN + 10, 0.5))
        return false;

    servoManager_->waitForTurret();
    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::MEDIUM_POSITION);
    robot_->wait(0.6);
    dropPlants(robot_, servoManager_, isDroppingFront_, 3 + zoneId_);
    robot_->getMotionController()->goStraight(-MARGIN);
    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);

    // Action should never be done again
    return true;
}


