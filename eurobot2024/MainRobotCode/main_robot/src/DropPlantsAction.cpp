#include "main_robot/DropPlantsAction.h"

#define CHASSIS_MARGIN 140
#define POT_MARGIN 125

/////////////////////////////////////
// Zone placement
//      5
//  _____________________
//  |1                  |
// 4|                   |
//  |                   | 2
//  |                   | 3
//  |0__________________|

const miam::RobotPosition PLANT_DROP_COORD[3] =
{
    miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 612, 0),
    miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 1388, 0),
    miam::RobotPosition(3000 - (CHASSIS_MARGIN + POT_MARGIN + 100), 612, M_PI)
};

const miam::RobotPosition PLANT_DROP_NOPOTS_COORD[3] =
{
    miam::RobotPosition(250, 250, M_PI),
    miam::RobotPosition(250, 2000 - 250, M_PI),
    miam::RobotPosition(3000 - 250, 1000, 0)
};

const miam::RobotPosition JARDINIERE_COORD[3] =
{
    // Jardiniere
    miam::RobotPosition(3000 - CHASSIS_MARGIN, 600, 0),
    miam::RobotPosition(CHASSIS_MARGIN, 1400, M_PI),
    miam::RobotPosition(760, 2000 - CHASSIS_MARGIN, M_PI_2)
};

const int DROP_PRIORITY_PREFERENCE[6] = {
    5,
    4,
    1,
    1,
    2,
    3
    };

void dropPlants(RobotInterface *robot, ServoManager *servos, bool dropFront, int zoneId, bool dropGround)
{
    servos->waitForTurret();
    if (dropGround)
    {
        servos->setClawPosition((dropFront ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::LOW_POSITION);
    }
    servos->openClaws(dropFront);
    robot->wait(0.5);

    int nPlants = -servos->updateClawContent(dropFront, robot->gameState_);
    robot->gameState_.nPlantsCollected[zoneId] += nPlants;
    robot->logger_ << "Dropped " << nPlants << " plants." << std::endl;

    // On ground, 2/3 of the plants are not valid -> each plant gives 1 point on average
    if (dropGround)
        robot->updateScore(nPlants, "plants dropped on ground");
    else
        robot->updateScore(2 * nPlants, "plants dropped in jardiniere"); // Heuristic: half the plants are valid
}

void DropPlantsWithPotAction::updateStartCondition()
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


void DropPlantsWithPotAction::actionStartTrigger()
{

    // Drop the most full claw only - or the one already placed.
    bool const isPlacedFront = std::abs(servoManager_->getTurretPosition()) < 0.1;

    int const clawDiff = robot_->gameState_.frontToBackClawDiff();
    if (clawDiff == 0)
    {
        // Don't move turret if both are the same.
        isDroppingFront_ = !isPlacedFront;
    }
    else
        isDroppingFront_ = clawDiff > 0;
    servoManager_->moveTurret(isDroppingFront_ ? M_PI : 0);
}

bool DropPlantsWithPotAction::performAction()
{
    //~ const miam::RobotPosition PLANT_DROP_COORD[3] =
    //~ {
        //~ miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 612, 0),
        //~ miam::RobotPosition(CHASSIS_MARGIN + POT_MARGIN + 100, 1388, 0),
        //~ miam::RobotPosition(3000 - (CHASSIS_MARGIN + POT_MARGIN + 100), 612, M_PI)
    //~ };

    // Push pots, moving backward.
    RobotPosition offset = RobotPosition(-50, 0, 0).rotate(PLANT_DROP_COORD[zoneId_].theta);
    offset.theta = 0;
    RobotPosition target = PLANT_DROP_COORD[zoneId_] + offset;
    if (!robot_->getMotionController()->goToStraightLine(target, 1.0, tf::BACKWARD))
        return false;

    // Turn on the magnets and close the arms to get the pots
    servoManager_->turnOnMagnets();
    servoManager_->closeElectromagnetArms();
    robot_->wait(0.4);
    if (!robot_->getMotionController()->goStraight(-50, 0.4)) // 0.4 = speed_ratio
        return false;
    servoManager_->openElectromagnetArms();
    robot_->wait(0.4);
    if (!robot_->getMotionController()->goStraight(50))
        return false;

    // Go to zone, pushing the pots
    target = robot_->getMotionController()->getCurrentPosition();
    std::vector<RobotPosition> positions;
    int yInvert = (zoneId_ == 0 ? -1 : 1);
    int xInvert = (zoneId_ == 2 ? -1 : 1);

    target.y += 350 * yInvert;
    positions.push_back(target);

    if (!robot_->getMotionController()->goToRoundedCorners(positions, 200, /*0.2*/ 0.1, static_cast<tf>(tf::BACKWARD | tf::IGNORE_END_ANGLE)))
    {
        // In the unlikely event of a failure here: drop everything in place.
        servoManager_->openElectromagnetArms();
        servoManager_->turnOffMagnets();
        return false;
    }

    // Drop plants
    servoManager_->turnOffFrontMagnets();

    int dropSign = (zoneId_ == 1 ? 1 : -1);
    if (robot_->isPlayingRightSide())
        dropSign = -dropSign;

    double turretOffset = (isDroppingFront_ ? M_PI : 0);
    int servoOffset = (isDroppingFront_ ? 0 : 3);

    // Drop plants in pots.
    servoManager_->moveTurret(turretOffset - dropSign * 0.2);
    robot_->wait(0.4);
    servoManager_->waitForTurret();
    servoManager_->openClaw(servoOffset + 1, false);
    robot_->wait(0.010);
    servoManager_->openClaw(servoOffset + (dropSign > 0 ? 2 : 0), false);
    robot_->wait(0.4);

    // Move back to drop in third pot
    if (!robot_->getMotionController()->goStraight(100))
        return true;

    // Move turret and drop
    servoManager_->moveTurret(turretOffset - dropSign * 0.65);
    robot_->wait(0.4);
    servoManager_->waitForTurret();
    servoManager_->openClaw(servoOffset + (dropSign > 0 ? 0 : 2), false);
    servoManager_->turnOffMagnets();

    robot_->wait(0.4);
    int nPlants = -servoManager_->updateClawContent(isDroppingFront_, robot_->gameState_);
    robot_->logger_ << "[DropPlantsWithPotAction] Dropped " << nPlants << " plants." << std::endl;
    robot_->updateScore(4 * nPlants, "plants dropped in pot");

    // Are there other plants to drop ?
    isDroppingFront_ = !isDroppingFront_;

    if (robot_->gameState_.nPlantsInRobot() > 0)
    {
        if (!robot_->getMotionController()->goStraight(150))
            return true;
        servoManager_->moveTurret(isDroppingFront_ ? 0 : M_PI);
        if (!robot_->getMotionController()->pointTurn(M_PI))
            return true;
        dropPlants(robot_, servoManager_, isDroppingFront_, zoneId_, true);
        if (!robot_->getMotionController()->goStraight(-120))
            return true;
        servoManager_->setClawPosition(isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK, ClawPosition::HIGH_POSITION);
        robot_->wait(0.8);
    }
    else
    {
        robot_->getMotionController()->goStraight(80);
    }

    return true;
}


void DropPlantsWithoutPotsAction::updateStartCondition()
{
    startPosition_ = PLANT_DROP_NOPOTS_COORD[zoneId_];
    isStartMotionBackward_ = false;

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


void DropPlantsWithoutPotsAction::actionStartTrigger()
{
    // drop front claw if there is something there.
    isDroppingFront_ = robot_->gameState_.nPlantsInClaw(true) > 0;
    servoManager_->moveTurret(isDroppingFront_ ? 0 : M_PI);
}

bool DropPlantsWithoutPotsAction::performAction()
{
    // Drop plants
    dropPlants(robot_, servoManager_, isDroppingFront_, zoneId_, true);
    robot_->getMotionController()->goStraight(-120);
    servoManager_->setClawPosition(isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK, ClawPosition::HIGH_POSITION);

    // Drop other plants, if needed
    isDroppingFront_ = !isDroppingFront_;
    if (robot_->gameState_.nPlantsInClaw(isDroppingFront_) > 0)
    {
        servoManager_->moveTurret(isDroppingFront_ ? 0 : M_PI);
        robot_->wait(0.5);
        dropPlants(robot_, servoManager_, isDroppingFront_, zoneId_, true);
        robot_->getMotionController()->goStraight(-150);
        servoManager_->setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    }

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
    // Drop the most full claw only - or the one already placed.
    bool const isPlacedFront = std::abs(servoManager_->getTurretPosition()) < 0.1;

    int const clawDiff = robot_->gameState_.frontToBackClawDiff();
    if (clawDiff == 0)
    {
        // Don't move turret if both are the same.
        isDroppingFront_ = isPlacedFront;
    }
    else
        isDroppingFront_ = clawDiff > 0;

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
    robot_->wait(1.0);
    dropPlants(robot_, servoManager_, isDroppingFront_, 3 + zoneId_);

    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::MEDIUM_POSITION_PLUS);
    robot_->wait(0.2);
    robot_->getMotionController()->goStraight(-MARGIN);
    servoManager_->setClawPosition((isDroppingFront_ ? ClawSide::FRONT : ClawSide::BACK), ClawPosition::HIGH_POSITION);

    // Action should never be done again
    return true;
}


