#include "main_robot/SolarPanelsAction.h"


#define LATERAL_DISTANCE 200

double PANELS_Y_COORD[6] = {270, 500, 725, 1275, 1500, 1725};

void SolarPanelsAction::updateStartCondition()
{
    startPosition_ = RobotPosition(PANELS_Y_COORD[0] - 60, LATERAL_DISTANCE, 0);

    priority_ = -1;

}


void SolarPanelsAction::actionStartTrigger()
{
    servoManager_->moveTurret(0);
}

bool SolarPanelsAction::performAction()
{
    robot_->logger_ << "[SolarPanelsAction] Performing action" << std::endl;

    servoManager_->waitForTurret();

    // TODO VLX...
    for (int i = 0; i< 6; i++)
    {
        RobotPosition const target(PANELS_Y_COORD[i], LATERAL_DISTANCE, 0);

        if (!robot_->getMotionController()->goToStraightLine(target, 1, false, true))
            return false;

        servoManager_->lowerSolarPanelArm();

        servoManager_->spinSolarPanel(true);
        robot_->wait(1);
        robot_->updateScore(5);
        servoManager_->spinSolarPanel(false);
        servoManager_->raiseSolarPanelArm();
    }

    // Action should not be done again
    return true;
}


