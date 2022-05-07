/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "Parameters.h"
#include "Strategy.h"

using namespace miam::trajectory;
using miam::RobotPosition;


// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;

void matchEndBackToBase(RobotInterface *robot)
{
    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
        MATCH_COMPLETED = true;
        // Compute falback trajectory.
        RobotPosition currentPosition = robot->getCurrentPosition();
        RobotPosition targetPosition;
        targetPosition.x = 950;
        targetPosition.y = 600;
        TrajectoryVector traj = computeTrajectoryStraightLineToPoint(currentPosition, targetPosition);
        robot->setTrajectoryToFollow(traj, true);
        if (robot->waitForTrajectoryFinished())
        {
            robot->updateScore(20);
        }
    }
}

Strategy::Strategy()
{
}

void Strategy::setup(RobotInterface *robot, ServoHandler *servo)
{
    this->robot = robot;
    this->servo = servo;

    servo->moveStatue(statue::FOLD);
    servo->activateMagnet(false);

    dropElements();

    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);
    servo->moveClaw(claw::FOLD);

    //init ventouse & rail
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    servo->moveSuction(1, suction::FOLD);

    if (robot->getTestMode())
    {
        robot->moveRail(0.5);
        robot->wait(2.0);
        robot->moveRail(0.65);
    }
}

void Strategy::match()
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

#ifndef SKIP_TO_GRABBING_SAMPLES

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    // Start fallback thread
#ifndef SIMULATION
    std::thread fallbackThread = std::thread(&matchEndBackToBase, robot);
    fallbackThread.detach();
#endif

    //**********************************************************
    // Go get the statue
    //**********************************************************
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    targetPosition.y = 450;
    positions.push_back(targetPosition);

    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.4);
    robot->setTrajectoryToFollow(traj);

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    robot->moveRail(0.7);
    servo->closeValve();
    robot->wait(0.5);
    servo->moveStatue(statue::TRANSPORT);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    if (wasMoveSuccessful)
        handleStatue();

    //**********************************************************
    // Go to the side distributor
    //**********************************************************
    moveSideSample();

    #endif

    #ifndef SKIP_TO_PUSHING_SAMPLES

    #ifdef SKIP_TO_GRABBING_SAMPLES

    targetPosition.x = 292;
    targetPosition.y = 1667 ;
    targetPosition.theta = 4.66973;
    robot->resetPosition(targetPosition, true, true, true);

    #endif

    //**********************************************************
    // Grab the three samples on the ground, and drop them
    //**********************************************************
    moveThreeSamples();

    std::cout << robot->getCurrentPosition() << std::endl;
    // robot->wait(1000);

    #endif

    #ifndef SKIP_TO_GRABBING_SAMPLES_SIDE_DIST

    #ifdef SKIP_TO_PUSHING_SAMPLES

    targetPosition.x = 766.546;
    targetPosition.y = 1635.35 ;
    targetPosition.theta = 7.8531;
    robot->resetPosition(targetPosition, true, true, true);

    #endif

    //**********************************************************
    // Push the samples on the field
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 1300;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.x = 950;
    targetPosition.y = 650;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    targetPosition.y = 450;
    positions.push_back(targetPosition);
    targetPosition.x = 350;
    targetPosition.y = 350;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    // Move the rail so it doesn't hit the fake statue
    robot->moveRail(0.9);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    usleep(2000000);
    servo->moveClaw(claw::SIDE);
    (void) robot->waitForTrajectoryFinished();
    robot->updateScore(15);

    // move back and fold arms
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -150);
    robot->setTrajectoryToFollow(traj);

    (void) robot->waitForTrajectoryFinished(); 

    servo->moveClaw(claw::FOLD);

    //**********************************************************
    // Flip the dig zone
    //**********************************************************
    handleDigZone();

    //**********************************************************
    // Go to the side tripledist
    //**********************************************************
    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);
    #endif

    handleSideTripleSamples();

    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 975;
    targetPosition.y = 600;
    targetPosition.theta = -M_PI_2;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    if (wasMoveSuccessful && !MATCH_COMPLETED)
    {
        robot->updateScore(20);
        MATCH_COMPLETED = true;
    }

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

