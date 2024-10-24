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
    servoManager_->raiseSolarPanelArm(true);
}

bool SolarPanelsAction::performAction()
{

    int const sign = robot_->isPlayingRightSide() ? -1 : 1;
    robot_->logger_ << "[SolarPanelsAction] Performing action" << std::endl;

    servoManager_->waitForTurret();
    servoManager_->raiseSolarPanelArm(true);
    robot_->wait(0.010);
    servoManager_->spinSolarPanel(robot_->isPlayingRightSide());
    robot_->wait(0.010);

    tf flags = (robot_->isPlayingRightSide() ? tf::DEFAULT : tf::BACKWARD);
    double finalAngle = (robot_->isPlayingRightSide() ? 0: M_PI);

    for (int i = 0; i< 6; i++)
    {
        if (i > 0)
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

            RobotPosition const target(PANELS_X_COORD[i] + CONTACT_ARM_OFFSET + ROBOT_ARM_OFFSET * sign, LATERAL_DISTANCE, finalAngle);
            std::cout << "Target position: " << target << std::endl;

            if (i == 3)
            {
                // Avoid pots which are present in this zone
                std::vector<RobotPosition> positions;
                positions.push_back(currentPosition);
                RobotPosition avoidPosition((currentPosition.x + target.x) / 2, currentPosition.y + 170, finalAngle);
                positions.push_back(avoidPosition);
                positions.push_back(target);
                TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(
                    robot_->getMotionController()->robotParams_.getTrajConf(),
                    positions,
                    100.0,
                    0.4,    // Transition velocity
                    flags
                );
                robot_->getMotionController()->setTrajectoryToFollow(traj);
                if (!robot_->getMotionController()->waitForTrajectoryFinished())
                    return true; // Never retry
            }
            else
                if (!robot_->getMotionController()->goToStraightLine(target, 1, flags))
                    return true; // Never retry
        }

        servoManager_->lowerSolarPanelArm();
        robot_->wait(0.010);
        servoManager_->lowerSolarPanelArm();
        robot_->wait(0.250);
        robot_->updateScore(5, "solar panel");
        servoManager_->raiseSolarPanelArm(true);
        robot_->wait(0.4);
    }
    servoManager_->raiseSolarPanelArm();
    servoManager_->stopSolarPanel();

    // Action should not be done again
    return true;
}


