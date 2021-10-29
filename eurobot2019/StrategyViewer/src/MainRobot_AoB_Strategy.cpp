/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/ArcCircle.h>

using namespace miam::trajectory;

// Define robot constants for trajectory generation.
namespace robotdimensions
{
    double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
    double const maxWheelSpeedTrajectory = 400; ///< Maximum wheel speed, in mm/s.
    double const maxWheelAccelerationTrajectory = 400; ///< Maximum wheel acceleration, in mm/s^2.
}

// Robot dimension.
double const CHASSIS_FRONT = 150.0;
double const CHASSIS_BACK = 150.0;
double const CHASSIS_WIDTH = 150.0;


void mainRobotAgeOfBotsStrategy(ViewerRobot &robot)
{
    robot.trajectory_.clear();
    robot.clearScore();
    std::cout << "Computing main robot strategy, obstacle at " << robot.obstacleX_ << " " << robot.obstacleY_ << std::endl;

    // Update config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    bool wasMoveSuccessful = true;

    // Set initial position
    targetPosition.x = CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot.resetPosition(targetPosition, true, true, true);

    //**********************************************************
    // Go get the statue.
    //**********************************************************
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    targetPosition.y = 450;
    positions.push_back(targetPosition);
    // Move at 45degree angle toward the statue
    targetPosition.x -= 80;
    targetPosition.y -= 80;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();
}

