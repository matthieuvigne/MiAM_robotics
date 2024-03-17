#include "main_robot/SolarPanelsAction.h"


#define LATERAL_DISTANCE 200

double PANELS_Y_COORD[6] = {270, 500, 725, 1275, 1500, 1725};

void SolarPanelsAction::updateStartCondition()
{
    startPosition_ = RobotPosition(PANELS_Y_COORD[0] - 40, LATERAL_DISTANCE, M_PI);
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
        RobotPosition const target(PANELS_Y_COORD[i], LATERAL_DISTANCE, M_PI);

        if (!robot_->getMotionController()->goToStraightLine(target, 1, true, true))
            return false;

        servoManager_->lowerSolarPanelArm();
        robot_->wait(0.4);

        servoManager_->spinSolarPanel(true);
        robot_->wait(0.5);
        robot_->updateScore(5);
        servoManager_->spinSolarPanel(false);
        servoManager_->raiseSolarPanelArm();
    }

    // Action should not be done again
    return true;
}


