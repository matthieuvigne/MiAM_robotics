/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/ArcCircle.h>

using miam::trajectory::rotationside;
using miam::trajectory::Trajectory;
using miam::trajectory::ArcCircle;

// Define robot constants for trajectory generation.
namespace robotdimensions
{
    double const wheelSpacing = 120.0; ///< Wheel spacing from robot center, in mm.
    double const maxWheelSpeed = 450; ///< Maximum wheel speed, in mm/s.
    double const maxWheelAcceleration = 650; ///< Maximum wheel acceleration, in mm/s^2.
}

// Robot dimension.
double const CHASSIS_FRONT = 46.0;
double const CHASSIS_BACK = 75.0;
double const CHASSIS_WIDTH = 169.0;


void secondaryRobotStrategy(ViewerRobot &robot)
{
    robot.trajectory_.clear();
    robot.clearScore();
    // Update config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeed,
                                                    robotdimensions::maxWheelAcceleration,
                                                    robotdimensions::wheelSpacing);

    // Starting position
    RobotPosition targetPosition;
    targetPosition.x = CHASSIS_WIDTH + 20.0;
    targetPosition.y = 1700 - CHASSIS_FRONT - 100.0;
    targetPosition.theta = G_PI_2;
    robot.resetPosition(targetPosition);

    RobotPosition resetPosition;
    std::vector<std::shared_ptr<miam::trajectory::Trajectory>> traj;
    std::vector<RobotPosition> positions;

    // Go get chaos zone, following a curved trajectory.
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 2000 - CHASSIS_WIDTH - 50;
    positions.push_back(targetPosition);
    targetPosition.x = 1000;
    positions.push_back(targetPosition);
    targetPosition.y = 1020;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.4);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Circle inside the chaos zone to grab all the atoms.
    std::shared_ptr<ArcCircle> circle(new ArcCircle(robot.getCurrentPosition(), 200.0, rotationside::RIGHT, - 1.2));
    traj.clear();
    traj.push_back(circle);

    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Go to the corner of the playing field, then up the ramp.
    positions.clear();
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    // Compute coordinate to generate no rotation and reach a point with x coordinate at 200.
    targetPosition.x = 200;
    double dx = targetPosition.x - robot.getCurrentPosition().x;
    targetPosition.y += dx * std::tan(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.y = 400;
    positions.push_back(targetPosition);
    targetPosition.y = 200;
    positions.push_back(targetPosition);
    targetPosition.x = 300;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.4);

    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Go hit back wall to reset position.
    targetPosition.x = CHASSIS_BACK - 10.0;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);

    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Reset position.
    targetPosition.theta = 0.0;
    targetPosition.y = robot.getCurrentPosition().y;
    robot.resetPosition(targetPosition, true, false, true);

    // Go push the atoms at the top of the ramp.
    targetPosition.x = 1228 - CHASSIS_FRONT - 30;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition);

    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);

    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    //~ robot.updateScore(32);
}
