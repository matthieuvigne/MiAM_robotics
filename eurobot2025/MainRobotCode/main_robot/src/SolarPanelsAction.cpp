#include "main_robot/SolarPanelsAction.h"


#define LATERAL_DISTANCE 170

double const PANELS_X_COORD[6] = {270, 500, 725, 1275, 1500, 1725};
double const ROBOT_ARM_OFFSET = 05.0;
double const CONTACT_ARM_OFFSET = 25.0;

void SolarPanelsAction::updateStartCondition()
{
    int const sign = robot_->isPlayingRightSide() ? -1 : 1;
    startPosition_ = RobotPosition(PANELS_X_COORD[0] + CONTACT_ARM_OFFSET + ROBOT_ARM_OFFSET * sign, LATERAL_DISTANCE, (robot_->isPlayingRightSide() ? 0: M_PI));
    isStartMotionBackward_ = !robot_->isPlayingRightSide();

    RobotPosition const robotStartPosition = robot_->getStartPosition();
    if (robotStartPosition.y < 500)
    {
        // Do this action first when starting on the lower left corner
        priority_ = 100.;
    }
    else
    {
        if (robot_->getMatchTime() < 20)
            priority_ = -1;
        else if (robot_->getMatchTime() > 50)
            priority_ = 25;
        else
            priority_ = 1;
    }
}


void SolarPanelsAction::actionStartTrigger()
{
    servoManager_->moveTurret(0);
}

bool SolarPanelsAction::performAction()
{

    // Action should not be done again
    return true;
}


