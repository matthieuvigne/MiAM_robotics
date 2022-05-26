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


#define USE_CAMERA 1

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
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);

    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);

    return robot->waitForTrajectoryFinished();
}


Strategy::Strategy()
{
    is_handle_statue_finished = false;
    is_move_side_sample_finished = false;
    is_handle_side_triple_samples_finished = false;
    is_move_three_samples_finished = false;
    is_handle_dig_zone_finished = false;
    is_bonus_already_counted = false;
    is_push_samples_below_shelter_finished = false;

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
    robot->updateScore(2);  //depose statuette
    robot->updateScore(2);  //depose vitrine


#ifndef SKIP_TO_GRABBING_SAMPLES

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    // start camera
    #if USE_CAMERA
    std::thread camThread(&network::CameraClient::run, &(camera_));
    camThread.detach();
    #endif

    //**********************************************************
    // Go get the statue
    //**********************************************************
    if (!handleStatue())
    {
        // Strategy fallback: alt 2

        stopEverything();

        // if failing to get the statue, the most probable reason should be sth
        // prevents us from getting to the vitrine
        // adverse robot should be near vitrine/side dist/presentoir
        // -> try go do dig sites

        targetPosition = robot->getCurrentPosition();
        if (targetPosition.y > 1100)
        {
            // try not to knock samples off starting zone
            endPosition = targetPosition;
            endPosition.x = 500;
            endPosition.y -= 200;
            traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
            robot->setTrajectoryToFollow(traj);
            robot->waitForTrajectoryFinished();

            if (!handleDigZone())
                stopEverything();

            if (!pushSamplesBelowShelter())
            {
                // before folding claws try to go back a little in order not to get a sample stuck
                // between claws
                // trajectory should be feasible given position
                targetPosition = robot->getCurrentPosition();
                traj = computeTrajectoryStraightLine(targetPosition, -150);
                robot->setTrajectoryToFollow(traj);
                robot->waitForTrajectoryFinished();
                stopEverything();
            }
            
            if (!handleSideTripleSamples())
                stopEverything();
        }
    }

    


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
    {
        // Strategy fallback: alt 1

        stopEverything();

        // go dig zone
        if (!handleDigZone())
        {
            // if failing dig zone, assume adverse robot
            // in the area and fallback to the side dist
            stopEverything();

            // go back not to knock samples
            targetPosition = robot->getCurrentPosition();
            RobotPosition endPosition = targetPosition;
            endPosition.x = 600;
            endPosition.y = robotdimensions::CHASSIS_WIDTH + 80 + 60 -10;
            traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);

            if (!handleSideTripleSamples())
                stopEverything();

            // then go back around 1100, 500 to get a better angle
            targetPosition = robot->getCurrentPosition();
            endPosition = targetPosition;
            endPosition.x = 450;
            endPosition.y = 1100;
            traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);   
            robot->setTrajectoryToFollow(traj);
            robot->waitForTrajectoryFinished();
        }

    }

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
    {
        // before folding claws try to go back a little in order not to get a sample stuck
        // between claws
        // trajectory should be feasible given position
        targetPosition = robot->getCurrentPosition();
        traj = computeTrajectoryStraightLine(targetPosition, -150);
        robot->setTrajectoryToFollow(traj);
        robot->waitForTrajectoryFinished();
        stopEverything();
    }

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
    // Strategy fallback: attempt moveThreeSamples if not done
    //**********************************************************

    if (!handleDigZone())
        stopEverything();
    if (!moveThreeSamplesBackup())
        stopEverything();


    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************

    servo->activatePump(false);
    MATCH_COMPLETED = goBackToDigSite();
    if (MATCH_COMPLETED)
    {
        robot->updateScore(20); // match completed
        camera_.shutDown();
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

    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(stratPtr);
    usleep(50000);
    servo->moveRail(robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;

        if (goBackToDigSite()) 
        {
            robot->updateScore(20); //match completed
            camera_.shutDown();
        }
        servo->activatePump(false);
    }
#endif
}

