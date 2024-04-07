#include "main_robot/SolarPanelsAction.h"


#define LATERAL_DISTANCE 170

double const PANELS_X_COORD[6] = {270, 500, 725, 1275, 1500, 1725};
double const ROBOT_ARM_OFFSET = 20.0;

void SolarPanelsAction::updateStartCondition()
{
    startPosition_ = RobotPosition(PANELS_X_COORD[0] - 40, LATERAL_DISTANCE, M_PI);
    if (robot_->getMatchTime() < 20)
        priority_ = -1;
    else if (robot_->getMatchTime() > 50)
        priority_ = 25;
    else
        priority_ = 1;
}


void SolarPanelsAction::actionStartTrigger()
{
    servoManager_->moveTurret(0);
}

bool SolarPanelsAction::performAction()
{
    robot_->logger_ << "[SolarPanelsAction] Performing action" << std::endl;

    servoManager_->waitForTurret();
    servoManager_->raiseSolarPanelArm(true);
    robot_->wait(0.010);
    servoManager_->spinSolarPanel(true);
    robot_->wait(0.010);

    for (int i = 0; i< 6; i++)
    {
        // Read VLX
        double const vlxY = robot_->getMeasurements()->vlxDistance;
        RobotPosition currentPosition = robot_->getMotionController()->getCurrentPosition();
        if (std::abs(currentPosition.y - vlxY) < 40.0)
        {
            robot_->logger_ << "[Strategy] Resetting robot y position: " << vlxY << " instead of " << currentPosition.y << std::endl;
            currentPosition.y = vlxY;
            robot_->getMotionController()->resetPosition(currentPosition, false, true, false);
        }

        RobotPosition const target(PANELS_X_COORD[i] + ROBOT_ARM_OFFSET, LATERAL_DISTANCE, M_PI);

        if (i == 3)
        {
            // Avoid pots which are present in this zone
            std::vector<RobotPosition> positions;
            positions.push_back(currentPosition);
            RobotPosition avoidPosition((currentPosition.x + target.x) / 2, currentPosition.y + 150, 0);
            positions.push_back(avoidPosition);
            positions.push_back(target);
            TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                robot_->getMotionController()->robotParams_.getTrajConf(),
                positions,
                100.0,
                0.2,    // Transition velocity
                tf::BACKWARD
            );
            robot_->getMotionController()->setTrajectoryToFollow(traj);
            if (!robot_->getMotionController()->waitForTrajectoryFinished())
                return false;
        }
        else
            if (!robot_->getMotionController()->goToStraightLine(target, 1, tf::BACKWARD))
                return false;

        servoManager_->lowerSolarPanelArm();
        robot_->wait(0.010);
        servoManager_->lowerSolarPanelArm();
        robot_->wait(0.3);
        robot_->updateScore(5);
        servoManager_->raiseSolarPanelArm(true);
        robot_->wait(0.4);
    }
    servoManager_->raiseSolarPanelArm();

    // Action should not be done again
    return true;
}


