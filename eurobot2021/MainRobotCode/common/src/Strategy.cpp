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

#define MOVE_OR_ABORT(a) if (!robot->waitForTrajectoryFinished()) { std::cout << a << std::endl; return;}

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

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
    if (robot->getTestMode())
    {
        robot->moveRail(0.5);
        robot->wait(2.0);
        robot->moveRail(0.65);
    }
}

bool shouldDrop(ExcavationSquareColor color, RobotInterface *robot)
{
    bool shouldDrop = false;
    if (robot->isPlayingRightSide())
    {
        shouldDrop = color ==  ExcavationSquareColor::PURPLE;
    }
    else
    {
        shouldDrop = color ==  ExcavationSquareColor::YELLOW;
    }
    return(shouldDrop);
}

void dropSite(RobotInterface *robot, ServoHandler *servo)
{
    servo->moveFinger(robot->isPlayingRightSide(), finger::PUSH);
    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    robot->wait(0.4);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
}


// Test an excavation site, pushing it if necessary.
ExcavationSquareColor testExcavationSite(RobotInterface *robot, ServoHandler *servo)
{
    // Take measurement
    servo->moveArm(robot->isPlayingRightSide(), arm::MEASURE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
    robot->wait(0.8);
    ExcavationSquareColor const color = robot->getExcavationReadings(robot->isPlayingRightSide());

    if (shouldDrop(color, robot))
    {
        dropSite(robot, servo);
        robot->updateScore(5);
    }
    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);

    return color;
}

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool IS_FALLBACK_MODE = false;
bool MATCH_COMPLETED = false;

void matchEndBackToBase(RobotInterface *robot)
{
    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    IS_FALLBACK_MODE = true;
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
        // Compute falback trajectory.
        RobotPosition currentPosition = robot->getCurrentPosition();
        RobotPosition targetPosition;
        targetPosition.x = 950;
        targetPosition.y = 600;
        TrajectoryVector traj = computeTrajectoryStraightLineToPoint(currentPosition, targetPosition);
        robot->setTrajectoryToFollow(traj);
        if (robot->waitForTrajectoryFinished())
        {
            robot->updateScore(20);
            MATCH_COMPLETED = true;
        }
    }
}

void setTraectoryToFollowFallback(RobotInterface *robot, TrajectoryVector const& traj)
{
    // If not in fallback, perform the action, otherwise just block everything.
    if (!IS_FALLBACK_MODE)
        robot->setTrajectoryToFollow(traj);
    else
        robot->wait(100);
}

// Handle the statue: grab it, swap, and drop the real statue.
// Abort as soon as one action fails.
void handleStatue(RobotInterface *robot, ServoHandler *servo)
{

    //Go back
    RobotPosition targetPosition = robot->getCurrentPosition();
    RobotPosition endPosition(230 + robotdimensions::CHASSIS_BACK,
                              230 + robotdimensions::CHASSIS_BACK,
                              0.0);

    TrajectoryVector traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition, 0.0, true);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveStatue(statue::CATCH);
    servo->activateMagnet(true);
    robot->wait(0.6);
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(5);

    //**********************************************************
    // Drop the fake statue
    //**********************************************************
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 140.0);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI)));
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 20.0);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    // Place fake statue
    robot->moveRail(0.4);
    servo->moveSuction(1, suction::DROP_FAKE_STATUE);
    dropElements(robot, servo);
    robot->wait(0.5);
    robot->updateScore(10);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -30.0);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    robot->moveRail(0.8);
    servo->moveSuction(1, suction::HORIZONTAL);

    //**********************************************************
    // Drop the real statue
    //**********************************************************
    std::vector<RobotPosition> positions;
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x += 80;
    targetPosition.y += 80;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_WIDTH + 100;
    targetPosition.y += 500;
    positions.push_back(targetPosition);
    targetPosition.y = 2000 - robotdimensions::CHASSIS_BACK - 5;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 300.0, 0.3, true);

    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->activateMagnet(false);
    servo->moveStatue(statue::DROP);
    robot->wait(0.1);
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(20);
}

void moveSideSample(RobotInterface *robot, ServoHandler *servo)
{
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1680 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 30;
    positions.push_back(targetPosition);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 300.0, 0.5);
    setTraectoryToFollowFallback(robot, traj);
    servo->moveStatue(statue::FOLD);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    servo->closeValve();
    robot->moveRail(0.55);
    robot->wait(1.0);
    robot->moveRail(0.8);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -150.0);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI_4)));
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    servo->activatePump(false);
    servo->openValve();
    robot->wait(1.0);
}

void handleSideTripleSamples(RobotInterface *robot, ServoHandler *servo)
{
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 200 + 40 + 10 + 20;
    targetPosition.y = 750;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 120 + 40 + 10 + 20;
    targetPosition.y = 750;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    // go grab the 1st sample
    servo->moveSuction(1, suction::LOWER_SAMPLE);
    servo->closeValve();
    servo->openTube(2);
    servo->closeTube(0);
    servo->closeTube(1);
    servo->activatePump(true);
    robot->moveRail(0.25);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 50);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    robot->wait(1.0);
    robot->moveRail(0.6);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -200);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI_2)));
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    dropElements(robot, servo);
    robot->wait(0.5);

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI_2)));
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    // go grab the 2d sample
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 230);
    setTraectoryToFollowFallback(robot, traj);

    servo->moveSuction(1, suction::LOWER_SAMPLE);
    servo->closeValve();
    servo->openTube(2);
    servo->closeTube(0);
    servo->closeTube(1);
    servo->activatePump(true);
    robot->moveRail(0.25);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
    robot->wait(1.0);
    robot->moveRail(0.6);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -130);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI_2)));
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    dropElements(robot, servo);
    robot->wait(0.5);
}


void moveThreeSamples(RobotInterface *robot, ServoHandler *servo)
{
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1325;
    positions.push_back(targetPosition);
    targetPosition.x = 650;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    setTraectoryToFollowFallback(robot, traj);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 60.0);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    servo->closeValve();
    for (int i = 0; i < 3; i++)
        servo->openTube(i);
    servo->activatePump(true);
    robot->moveRail(0.0);
    robot->wait(1.0);

    robot->moveRail(0.75);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::VERTICAL);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    targetPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 70 - 30;
    targetPosition.theta = M_PI_2;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    setTraectoryToFollowFallback(robot, traj);
    robot->wait(1.0);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::LOWER_SAMPLE);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    dropElements(robot, servo);
    robot->moveRail(0.3);
    robot->wait(2.0);
    robot->updateScore(9);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -90);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");
}

void handleDigZone(RobotInterface *robot, ServoHandler *servo)
{
    const int spacing_between_sites = 185;
    const int site_y = robotdimensions::CHASSIS_WIDTH + 80 + 60 -10;
    const int first_site_x = 730 - 15;

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x -= 100 * std::cos(targetPosition.theta);
    targetPosition.y -= 100 * std::sin(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.x = 620;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleDigZone failed to complete");

    std::cout << "First correction" << std::endl;
    // Check with range sensor: are we at the right place ?
    double const MAXIMUM_RANGE_CORRECTION = 100;

    robot->wait(0.05);
    double measured_y = robot->getRangeSensorMeasurement(robot->isPlayingRightSide());
    targetPosition = robot->getCurrentPosition();
    double sumOfAllCorrections = 0;
    if (std::abs(sumOfAllCorrections + measured_y - targetPosition.y) < MAXIMUM_RANGE_CORRECTION)
    {
        std::cout << "Resetting position thanks to range sensor. Error:" << targetPosition.y - measured_y << std::endl;
        sumOfAllCorrections += measured_y - targetPosition.y;
        targetPosition.y = measured_y;
        robot->resetPosition(targetPosition, false, true, false);
    }
    std::cout << "After reset position ; error:" << targetPosition.y - measured_y << std::endl;

    positions.clear();
    positions.push_back(targetPosition);

    targetPosition.x = 670;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);

    targetPosition.x = first_site_x;
    positions.push_back(targetPosition);

    traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.3, true);
    setTraectoryToFollowFallback(robot, traj);
    MOVE_OR_ABORT("handleDigZone failed to complete");

    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);

    // Test all sites.
    testExcavationSite(robot, servo);
    // Test the first 3 sites
    for (int i = 1; i < 3; i++)
    {
        // Check range
        measured_y = robot->getRangeSensorMeasurement(robot->isPlayingRightSide());
        targetPosition = robot->getCurrentPosition();
        if (std::abs(sumOfAllCorrections + measured_y - targetPosition.y) < MAXIMUM_RANGE_CORRECTION)
        {
            std::cout << "Resetting position thanks to range sensor. Error:" << targetPosition.y - measured_y << std::endl;
            sumOfAllCorrections += measured_y - targetPosition.y;
            targetPosition.y = measured_y;
            robot->resetPosition(targetPosition, false, true, false);
        }
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = first_site_x + (i - 0.5) * spacing_between_sites;
        targetPosition.y = site_y;
        positions.push_back(targetPosition);
        targetPosition.x = first_site_x + i * spacing_between_sites;
        targetPosition.y = site_y;
        positions.push_back(targetPosition);
        traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.3, true);
        setTraectoryToFollowFallback(robot, traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");
        testExcavationSite(robot, servo);
    }

    // positions of the 4th, 5th, 6th, 7th sites
    // int site_positions[] = {
    //     first_site_x + 3 * spacing_between_sites,
    //     first_site_x + 4 * spacing_between_sites,
    //     first_site_x + 5 * spacing_between_sites,
    //     first_site_x + 6 * spacing_between_sites};

    int targeted_site = 0;
    bool know_targeted_site_is_ours = false;

    // Then test the 4 sites ; infer if possible
    while (targeted_site < 4)
    {

        // go to site
        targetPosition.x = first_site_x + (targeted_site + 3) * spacing_between_sites;
        targetPosition.y = site_y;
        traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition, 0.0, true);
        setTraectoryToFollowFallback(robot, traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");

        if (know_targeted_site_is_ours)
        {
            // bascule
            dropSite(robot, servo);
            robot->updateScore(5);

            // our sites are 0 and 3 (normally this condition is useless)
            if (targeted_site == 0)
            {
                targeted_site = 3;
                continue;
            }
            // our sites are 1 and 2
            else if (targeted_site == 1)
            {
                targeted_site = 2;
                continue;
            // if our site was 2 or 3 then job is done
            } else {
                // break the loop
                break;
            }
        }
        else
        {
            // measure and resolve
            ExcavationSquareColor color = testExcavationSite(robot, servo);

            // if measurement is successful then we know where our sites are
            if (color != ExcavationSquareColor::NONE) {
                // if we found our site
                if (shouldDrop(color, robot)) {
                    // our sites are 0 and 3
                    if (targeted_site == 0)
                    {
                        targeted_site = 3;
                        know_targeted_site_is_ours = true;
                        continue;
                    }
                    // our site are 1 and 2
                    else if (targeted_site == 1) {
                        targeted_site = 2;
                        know_targeted_site_is_ours = true;
                        continue;
                    }
                    // if reached 2 or 3, job done
                    else
                    {
                        break;
                    }
                }
                // if we did not find our site
                else
                {
                    // site 0 is opposite team so our sites are 1 and 2
                    if (targeted_site == 0)
                    {
                        targeted_site = 1;
                        know_targeted_site_is_ours = true;
                        continue;
                    }
                    // site 1 is opposite team so our sites are 0 and 3
                    else if (targeted_site == 1) {
                        targeted_site = 3;
                        know_targeted_site_is_ours = true;
                        continue;
                    }
                    // site 2 is opposite team so our sites are 0 and 3
                    else if (targeted_site == 2)
                    {
                        targeted_site = 3;
                        know_targeted_site_is_ours = true;
                        continue;
                    }
                    // job done
                    else
                    {
                        break;
                    }
                }
            }
        }

        targeted_site++;
    }
    robot->updateScore(5);
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
#endif

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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.4);
    setTraectoryToFollowFallback(robot, traj);

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    robot->moveRail(0.7);
    servo->closeValve();
    robot->wait(0.5);
    servo->moveStatue(statue::TRANSPORT);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    if (wasMoveSuccessful)
        handleStatue(robot, servo);

    //**********************************************************
    // Go to the side distributor
    //**********************************************************
    moveSideSample(robot, servo);

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
    moveThreeSamples(robot, servo);

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
    setTraectoryToFollowFallback(robot, traj);
    // Move the rail so it doesn't hit the fake statue
    robot->moveRail(0.9);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    (void) robot->waitForTrajectoryFinished();
    robot->updateScore(15);

    //**********************************************************
    // Flip the dig zone
    //**********************************************************
    handleDigZone(robot, servo);

    //**********************************************************
    // Go to the side tripledist
    //**********************************************************
    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);
    #endif

    handleSideTripleSamples(robot, servo);






    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 975;
    targetPosition.y = 600;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    setTraectoryToFollowFallback(robot, traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    if (wasMoveSuccessful)
    {
        robot->updateScore(20);
        MATCH_COMPLETED = true;
    }

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

