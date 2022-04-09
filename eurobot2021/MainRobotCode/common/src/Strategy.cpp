/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "Parameters.h"
#include "Strategy.h"

using namespace miam::trajectory;
using miam::RobotPosition;

// Drop everything from the suction system
void dropElements(RobotInterface *robot, ServoHandler *servo)
{
    servo->activatePump(false);
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(0.5);
}

void setupRobot(RobotInterface *robot, ServoHandler *servo)
{
    servo->moveStatue(statue::FOLD);
    servo->activateMagnet(false);

    dropElements(robot, servo);

    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);

    //init ventouse & rail
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    servo->moveSuction(1, suction::HOLD_FAKE_STATUE);
    robot->moveRail(0.5);
    robot->wait(2.0);
    robot->moveRail(0.65);
}

// Test an excavation site, pushing it if necessary.
void testExcavationSite(RobotInterface *robot, ServoHandler *servo)
{
    // Take measurement
    servo->moveArm(robot->isPlayingRightSide(), arm::MEASURE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
    robot->wait(0.8);
    ExcavationSquareColor const color = robot->getExcavationReadings(robot->isPlayingRightSide());
    bool shouldDrop = false;
    if (robot->isPlayingRightSide())
    {
        shouldDrop = color ==  ExcavationSquareColor::PURPLE;
    }
    else
    {
        shouldDrop = color ==  ExcavationSquareColor::YELLOW;
    }
    if (shouldDrop)
    {
        servo->moveFinger(robot->isPlayingRightSide(), finger::PUSH);
        servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
        robot->updateScore(5);
        robot->wait(0.4);
        servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
    }
    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
}

void matchStrategy(RobotInterface *robot, ServoHandler *servo)
{
    std::cout << "Strategy thread started." << std::endl;

    // Update config.
    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    bool wasMoveSuccessful = true;
    robot->updateScore(2);

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    //**********************************************************
    // Go get the statue
    //**********************************************************
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 350;
    positions.push_back(targetPosition);
    targetPosition.y = 430 + 80;
    positions.push_back(targetPosition);

    // Move at 45degree angle toward the statue
    targetPosition.x += 80;
    targetPosition.y -= 80;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    servo->closeValve();
    robot->wait(0.5);
    servo->moveStatue(statue::TRANSPORT);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //Go back

    targetPosition = robot->getCurrentPosition();
    endPosition.x = 230 + robotdimensions::CHASSIS_BACK;
    endPosition.y = 230 + robotdimensions::CHASSIS_BACK;
    traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition, 0.0, true);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->moveStatue(statue::CATCH);
    servo->activateMagnet(true);
    robot->wait(0.8);
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(5);

    //**********************************************************
    // Drop the fake statue
    //**********************************************************

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 140.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 20.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    // Place fake statue
    robot->moveRail(0.4);
    servo->moveSuction(1, suction::DROP_FAKE_STATUE);
    dropElements(robot, servo);
    robot->wait(1.0);
    robot->updateScore(10);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -30.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //**********************************************************
    // Drop the real statue
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x += 80;
    targetPosition.y += 80;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_WIDTH + 100;
    targetPosition.y += 500;
    positions.push_back(targetPosition);
    targetPosition.y = 2000 - robotdimensions::CHASSIS_BACK - 20;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 300.0, 0.3, true);

    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->activateMagnet(false);
    servo->moveStatue(statue::DROP);
    robot->wait(0.1);
    robot->updateScore(20);

    //**********************************************************
    // Grab the three samples on the ground, and drop them
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1325;
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    robot->wait(1.5);
    servo->moveStatue(statue::FOLD);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);

    robot->wait(0.5);
    robot->moveRail(0.05);

    servo->closeValve();
    servo->activatePump(true);
    robot->wait(1.0);

    robot->moveRail(0.75);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::LOWER_SAMPLE);

    targetPosition = robot->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 70 - 5;
    traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    dropElements(robot, servo);
    robot->moveRail(0.5);

    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, -15);
    // robot->setTrajectoryToFollow(traj);
    // wasMoveSuccessful = robot->waitForTrajectoryFinished();
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::DROP_SAMPLE);
    robot->wait(0.3);
    robot->updateScore(9);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -90);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //**********************************************************
    // Push the samples on the field
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 1200;
    targetPosition.y = 800 ;
    positions.push_back(targetPosition);
    targetPosition.x = 800;
    targetPosition.y = 450 ;
    positions.push_back(targetPosition);
    targetPosition.x = 460 ;
    targetPosition.y = 230 ;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    // Move the rail so it doesn't hit the fake statue
    robot->moveRail(0.9);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(15);

    //**********************************************************
    // Flip the dig zone
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x -= 100 * std::cos(targetPosition.theta);
    targetPosition.y -= 100 * std::sin(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.x = 620;
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 80 + 60;
    positions.push_back(targetPosition);
    targetPosition.x = 730;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);

    // Test all sites.
    testExcavationSite(robot, servo);
    for (int i = 0; i < 6; i++)
    {
        targetPosition.x = 730 + (i + 1) * 185;
        traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition, 0.0, true);
        robot->setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot->waitForTrajectoryFinished();
        testExcavationSite(robot, servo);
    }
    robot->updateScore(5);

    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 900;
    targetPosition.y = 600;
    positions.push_back(targetPosition);
    targetPosition.x = 975;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(20);

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

