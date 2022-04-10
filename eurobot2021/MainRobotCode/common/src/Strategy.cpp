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

#define SKIP_TO_GRABBING_SAMPLES 1
#define SKIP_TO_PUSHING_SAMPLES 1
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
    robot->moveRail(0.5);
    robot->wait(2.0);
    robot->moveRail(0.65);
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
    robot->moveRail(0.7);
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

    robot->moveRail(0.8);
    servo->moveSuction(1, suction::HORIZONTAL);

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
    // Go to the side distributor
    //**********************************************************

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1680 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 30;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 300.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->moveStatue(statue::FOLD);
    // servo->turnOnPump();
    //servo->moveSuction(false);
    // robot->updateScore(1);
    
    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    servo->closeValve();
    robot->moveRail(0.55);
    robot->wait(1.0);
    robot->moveRail(0.8);
    
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -150.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    robot->moveRail(0.4);

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI_2)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 200.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->activatePump(false);
    servo->openValve();
    robot->wait(1.0);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -200.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    std::cout<< robot->getCurrentPosition() << std::endl;
    robot->wait(5.0);
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
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1325;
    positions.push_back(targetPosition);
    targetPosition.x = 650;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    

    // last straight line should be slower
    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory / 2.0,
                                                robotdimensions::maxWheelAccelerationTrajectory,
                                                robotdimensions::wheelSpacing);

    // positions.clear();
    // targetPosition = robot->getCurrentPosition();
    // positions.push_back(targetPosition);
    // targetPosition.x = 770;
    // positions.push_back(targetPosition);
    // traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 60.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);

    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                robotdimensions::maxWheelAccelerationTrajectory,
                                                robotdimensions::wheelSpacing);

    robot->wait(0.5);
    robot->moveRail(0.05);

    servo->closeValve();
    servo->activatePump(true);
    robot->wait(1.0);

    // robot->moveRail(0.75);
    // for (int i = 0; i < 3; i++)
    //     servo->moveSuction(i, suction::LOWER_SAMPLE);

    robot->moveRail(0.75);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::VERTICAL);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    // targetPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 70 - 25;
    // targetPosition.theta = M_PI_2;
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    targetPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 70 - 25;
    targetPosition.theta = M_PI_2;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    dropElements(robot, servo);
    robot->moveRail(0.5);

    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, -15);
    // robot->setTrajectoryToFollow(traj);
    // wasMoveSuccessful = robot->waitForTrajectoryFinished();


    // for (int i = 0; i < 3; i++)
    //     servo->moveSuction(i, suction::DROP_SAMPLE);
    // robot->wait(0.3);


    robot->wait(2.0);
    robot->updateScore(9);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -90);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    std::cout<< robot->getCurrentPosition() << std::endl;
    robot->wait(10);

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

    const int spacing_between_sites = 185;
    const int site_y = robotdimensions::CHASSIS_WIDTH + 80 + 60 -10;
    const int first_site_x = 730 - 15;

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x -= 100 * std::cos(targetPosition.theta);
    targetPosition.y -= 100 * std::sin(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.x = 620;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    targetPosition.x = first_site_x;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);

    // Test all sites.
    testExcavationSite(robot, servo);
    // for (int i = 0; i < 6; i++)
    // {
    //     targetPosition.x = 730 + (i + 1) * 185;
    //     traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition, 0.0, true);
    //     robot->setTrajectoryToFollow(traj);
    //     wasMoveSuccessful = robot->waitForTrajectoryFinished();
    //     testExcavationSite(robot, servo);
    // }
    // robot->updateScore(5);

    // Test the first 3 sites
    for (int i = 1; i < 3; i++)
    {
        targetPosition.x = first_site_x + i * spacing_between_sites;
        targetPosition.y = site_y;
        traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition, 0.0, true);
        robot->setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot->waitForTrajectoryFinished();
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
        robot->setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot->waitForTrajectoryFinished();

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

    //**********************************************************
    // Go to the side tripledist
    //**********************************************************
    

    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);


    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 200 + 40 + 10 + 20;
    targetPosition.y = 1200 - 360 + 75;           
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 120 + 40 + 10 + 20;
    targetPosition.y = 1200 - 360 + 75;           
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    #endif


    servo->moveSuction(2, suction::DROP_SAMPLE);

    robot->moveRail(0.25);
    robot->wait(1.0);

    servo->closeValve();
    servo->openTube(2);
    servo->closeTube(0);
    servo->closeTube(1);
    servo->activatePump(true);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 50);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    robot->wait(3.0);

    robot->moveRail(0.5);
    // robot->wait(2.0);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -200);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI_2)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    dropElements(robot, servo);
    robot->wait(1.0);

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI_2)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 200);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->moveSuction(2, suction::DROP_SAMPLE);

    robot->moveRail(0.25);
    robot->wait(1.0);

    servo->closeValve();
    servo->openTube(2);
    servo->closeTube(0);
    servo->closeTube(1);
    servo->activatePump(true);
    
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 60);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    robot->wait(3.0);

    robot->moveRail(0.5);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -180);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI_2)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    dropElements(robot, servo);
    robot->wait(1.0);


    // //**********************************************************
    // // Rotate to come back to the campment
    // //**********************************************************
    // positions.clear();
    // targetPosition = robot->getCurrentPosition();
    // positions.push_back(targetPosition);
    // targetPosition.x = 975;
    // targetPosition.y = 600;
    // positions.push_back(targetPosition);
    // traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    // robot->setTrajectoryToFollow(traj);
    // wasMoveSuccessful = robot->waitForTrajectoryFinished();
    // robot->updateScore(20);

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

