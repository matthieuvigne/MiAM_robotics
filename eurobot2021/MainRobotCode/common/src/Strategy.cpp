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



bool Strategy::goBackToDigSite()
{
    RobotPosition targetPosition;
    std::vector<RobotPosition> positions;

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 925;
    targetPosition.y = 620;
    positions.push_back(targetPosition);
    targetPosition.x = 930;
    targetPosition.y = 620;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);


    return robot->waitForTrajectoryFinished();
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

void Strategy::match_impl()
{

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    robot->updateScore(2);


#ifndef SKIP_TO_GRABBING_SAMPLES

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    //**********************************************************
    // Go get the statue
    //**********************************************************
    if (!handleStatue())
        stopEverything();

    //**********************************************************
    // Go to the side distributor
    //**********************************************************
    if (!moveSideSample())
        stopEverything();

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
    if (!moveThreeSamples())
        stopEverything();

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
    if (!pushSamplesBelowShelter())
        stopEverything();

    //**********************************************************
    // Flip the dig zone
    //**********************************************************
    if (!handleDigZone())
        stopEverything();

    //**********************************************************
    // Go to the side tripledist
    //**********************************************************
    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);
    #endif

    if (!handleSideTripleSamples())
        stopEverything();

    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************
    MATCH_COMPLETED = goBackToDigSite();
    if (MATCH_COMPLETED)
    {
        robot->updateScore(20);
    }

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    // Update config.
    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

#ifdef SIMULATION
        match_impl();
#else
    std::thread stratMain(&Strategy::match_impl, this);
    auto stratPtr = stratMain.native_handle();
    stratMain.detach();

    double const FALLBACK_TIME = 3.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
    {
        pthread_cancel(stratPtr);
        usleep(50000);
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;

        if (goBackToDigSite())
            robot->updateScore(20);

        servo->activatePump(false);
    }
#endif
}

