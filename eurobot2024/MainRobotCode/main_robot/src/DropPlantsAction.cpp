#include "main_robot/DropPlantsAction.h"



const miam::RobotPosition PLANT_DROP_COORD[6] =
{
        miam::RobotPosition(800, 1700, M_PI_2),
        miam::RobotPosition(300, 1600, M_PI_2),
        miam::RobotPosition(250, 1400, M_PI),
        miam::RobotPosition(2785, 1000),
        miam::RobotPosition(2785, 600),
        miam::RobotPosition(250, 150, M_PI),
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
    servoManager_->setClawPosition(ClawPosition::LOW_POSITION);
    robot_->wait(0.5);
    servoManager_->openClaws(true);
    robot_->gameState_.nPlantsCollected[zoneId_] += robot_->gameState_.nPlantsInRobot();

    for (int i = 0; i < 6; i++)
        robot_->gameState_.robotClawContent[i] = ClawContent::EMPTY;

    robot_->getMotionController()->goStraight(-100);

    // Action should not be done again if there are more than 3 plants.
    return robot_->gameState_.nPlantsCollected[zoneId_] > 3;
}


