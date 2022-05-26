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

#define MOVE_OR_ABORT(a) if (!robot->waitForTrajectoryFinished()) { std::cout << a << std::endl; return(false);}

bool samples_are_knocked_out[] = {false, false, false, false, false, false, false, false};

// Handle the statue: grab it, swap, and drop the real statue.
// Abort as soon as one action fails.
bool Strategy::handleStatue()
{
    if (is_handle_statue_finished) 
    {
        return(true);
    }

    std::vector<RobotPosition> positions;

    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    targetPosition.y = 450;
    positions.push_back(targetPosition);

    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.4);
    robot->setTrajectoryToFollow(traj);

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    robot->moveRail(0.7);
    servo->openValve();
    robot->wait(0.5);
    servo->moveStatue(statue::TRANSPORT);
    MOVE_OR_ABORT("handleStatue failed to complete");

    //Go back
    targetPosition = robot->getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 250 + robotdimensions::CHASSIS_BACK;
    targetPosition.y = 250 + robotdimensions::CHASSIS_BACK;
    positions.push_back(targetPosition);
    targetPosition.x = 225 + robotdimensions::CHASSIS_BACK;
    targetPosition.y = 225 + robotdimensions::CHASSIS_BACK;
    positions.push_back(targetPosition);

    traj = computeTrajectoryRoundedCorner(positions, 200, 0.3, true);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveStatue(statue::CATCH);
    servo->activateMagnet(true);
    robot->wait(0.6);
    std::cout << robot->getCurrentPosition() << std::endl;
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(3); // statue not on the piedestral 5 = 2 + 3

    //**********************************************************
    // Drop the fake statue
    //**********************************************************
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 140.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI)));
    robot->setTrajectoryToFollow(traj);
    robot->moveRail(0.1);
    MOVE_OR_ABORT("handleStatue failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 50.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    // Place fake statue
    servo->moveSuction(1, suction::DROP_STATUE);
    robot->wait(0.5); // wait to avoid dropping too brutally
    dropElements();
    robot->wait(0.2);
    robot->updateScore(5); // dropped fake statue 10 = 5 + 5

    // get the 2 samples on the side of the statue
    robot->moveRail(0.8);
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 80.0);
    robot->setTrajectoryToFollow(traj);


    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveSuction(0, suction::HORIZONTAL);
    servo->moveSuction(2, suction::HORIZONTAL);
    servo->moveSuction(1, suction::DROP_STATUE);

    servo->openTube(0);
    servo->closeTube(1);
    servo->openTube(2);
    servo->activatePump(true);
    servo->openValve();
    robot->moveRail(0.58);
    robot->wait(1.0);
    robot->moveRail(0.9);
    robot->updateScore(2); //2 samples moved from the shelter

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -50.0);
    robot->setTrajectoryToFollow(traj);

    MOVE_OR_ABORT("handleStatue failed to complete");

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
    targetPosition.y += 600;
    positions.push_back(targetPosition);
    targetPosition.y = 2000 - robotdimensions::CHASSIS_BACK - 5;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 300.0, 0.3, true);
    robot->setTrajectoryToFollow(traj);

    // drop samples when y > 1200
    while (!robot->isTrajectoryFinished()) {
        targetPosition = robot->getCurrentPosition();
        if (targetPosition.y > 1000) {
            servo->activatePump(false);
            servo->openValve() ;
            servo->openTube(0);
            servo->openTube(1);
            servo->openTube(2);
            // add 2 points
            robot->updateScore(2);
            break;
        }
    }
    robot->updateScore(2); //2 samples dropped

    // raise back rail and suction
    robot->moveRail(0.8);
    servo->moveSuction(1, suction::HORIZONTAL);

    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveStatue(statue::DROP);
    robot->wait(0.1);
    servo->activateMagnet(false);
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(5);  //the statue is in the display 15 = 10 + 5
    robot->updateScore(5);  //the display is activated

    is_handle_statue_finished = true;
    return(true);
}

bool Strategy::moveSideSample()
{
    if (is_move_side_sample_finished) 
    {
        return(true);
    }

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1680 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 20;
    positions.push_back(targetPosition);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 300.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    robot->wait(1.0);
    servo->moveStatue(statue::FOLD);
    // if suction not in the right place, do
    servo->moveSuction(1, suction::HORIZONTAL);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    servo->openValve();
    robot->moveRail(0.58);
    robot->wait(1.0);
    robot->moveRail(0.8);
    
    robot->updateScore(1);  //sample is moved from the distributor

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -150.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI_4)));
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    servo->activatePump(false);
    servo->openValve();
    robot->moveRail(0.3);

    is_move_side_sample_finished = true;
    robot->updateScore(1);  //sample is droped in the campment
    return(true);
}

bool Strategy::handleSideTripleSamples()
{
    if (is_handle_side_triple_samples_finished) 
    {
        return(true);
    }

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 270;
    targetPosition.y = 750 + 5;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 190;
    targetPosition.y = 750 + 5;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    robot->moveRail(0.25);
    servo->moveSuction(0, suction::FOLD);
    servo->moveSuction(2, suction::FOLD);
    servo->moveSuction(1, suction::LOWER_SAMPLE);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    // Grab and drop the three samples
    for (int i = 0; i < 3; i++)
    {
        servo->openValve();
        servo->openTube(2);
        servo->closeTube(0);
        servo->closeTube(1);
        servo->activatePump(true);

        targetPosition = robot->getCurrentPosition();
        RobotPosition endPosition = targetPosition;
        endPosition.x = robotdimensions::SUCTION_CENTER + 80 - 20 * i;
        traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
        robot->moveRail(0.25);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        robot->wait(0.5);
        robot->moveRail(0.6);
        robot->updateScore(1);  //sample is removed from distributor
        targetPosition = robot->getCurrentPosition();
        if (i == 2)
        {
            // last sample go back a little more to avoid hitting dist
            traj = computeTrajectoryStraightLine(targetPosition, -65);
            robot->setTrajectoryToFollow(traj);
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        } else 
        {
            traj = computeTrajectoryStraightLine(targetPosition, -45);
            robot->setTrajectoryToFollow(traj);
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        }

        // rotate -80 degres and move forward 70mm
        targetPosition = robot->getCurrentPosition();
        traj.clear();
        traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI * 80.0 / 180.0)));
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        targetPosition = robot->getCurrentPosition();
        robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, 70));
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        dropElements();
        robot->updateScore(1);  //sample is droped in the campment
        robot->wait(0.3);
        servo->moveSuction(1, suction::LOWER_SAMPLE);

        // go a little further if last movement
        if (i == 2) {
            targetPosition = robot->getCurrentPosition();
            robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, -70));
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

            targetPosition = robot->getCurrentPosition();
            traj.clear();
            traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI * 80.0 / 180.0)));
            robot->setTrajectoryToFollow(traj);
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

            targetPosition = robot->getCurrentPosition();
            robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, -250));
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

        } else {
            // inverse movement
            targetPosition = robot->getCurrentPosition();
            robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, -70));
            MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
        }

        // targetPosition = robot->getCurrentPosition();
        // positions.clear();
        // positions.push_back(targetPosition);
        // targetPosition.y += 70;
        // positions.push_back(targetPosition);
        // traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
        // robot->setTrajectoryToFollow(traj);
        // MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

        // dropElements();
        // robot->wait(0.3);
        // servo->moveSuction(1, suction::LOWER_SAMPLE);
        // targetPosition = robot->getCurrentPosition();
        // robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, -70));
        // MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
    }

    is_handle_side_triple_samples_finished = true;
    return(true);
}

bool Strategy::moveThreeSamples()
{
    if (is_move_three_samples_finished) 
    {
        return(true);
    }

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1325;
    positions.push_back(targetPosition);
    targetPosition.x = 650;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 65.0);
    robot->setTrajectoryToFollow(traj);
    servo->moveClaw(claw::VPOSITION); // move claws to push center sample a little more
    MOVE_OR_ABORT("moveThreeSamples failed to complete");
    servo->moveClaw(claw::FOLD);

    servo->closeValve();
    for (int i = 0; i < 3; i++)
        servo->openTube(i);
    servo->activatePump(true);
    robot->moveRail(0.0);
    robot->wait(1.0);

    // the samples are not there anymore
    is_move_three_samples_finished = true;

    // raise the rail a little before beginning to move
    robot->moveRail(0.25);

    // move and simultaneously raise rail and set suction vertical
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    positions.push_back(targetPosition);
    targetPosition.x = 770;
    targetPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 95 - 3;
    targetPosition.theta = M_PI_2;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);

    // raise rail and travel
    robot->moveRail(0.75);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::VERTICAL);

    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    // drop the samples
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::LOWER_SAMPLE);
    dropElements();
    
    robot->wait(1.0);
    robot->moveRail(0.35);
    robot->wait(1.0);

    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::DROP_SAMPLE);

    robot->wait(1.0); // wait a little longer to drop samples correctly
    robot->updateScore(9); // samples in the gallery

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -140);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    return(true);
}


bool Strategy::moveThreeSamplesBackup()
{
    std::cout << ">>> performing moveThreeSamplesBackup" << std::endl;
    if (is_move_three_samples_finished) 
    {
        return(true);
    }

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();

    // are we at the left of the samples ?
    bool to_the_left_of_the_samples = targetPosition.x < 800;

    positions.push_back(targetPosition);
    targetPosition.y = 1325;
    positions.push_back(targetPosition);
    // targetPosition.x = 625;
    // positions.push_back(targetPosition);

    if (to_the_left_of_the_samples)
    {
        targetPosition.x = 625;
        targetPosition.y = 1320;
        positions.push_back(targetPosition);
        targetPosition.x = 650;
        targetPosition.y = 1325;
        positions.push_back(targetPosition);
    } else 
    {
        targetPosition.x = 1175;
        targetPosition.y = 1320;
        positions.push_back(targetPosition);
        targetPosition.x = 1150;
        targetPosition.y = 1325;
        positions.push_back(targetPosition);
    }

    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
    robot->setTrajectoryToFollow(traj);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");


    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 65.0);
    robot->setTrajectoryToFollow(traj);
    servo->moveClaw(claw::VPOSITION); // move claws to push center sample a little more
    MOVE_OR_ABORT("moveThreeSamples failed to complete");
    servo->moveClaw(claw::FOLD);

    servo->closeValve();
    for (int i = 0; i < 3; i++)
        servo->openTube(i);
    servo->activatePump(true);
    robot->moveRail(0.0);
    robot->wait(1.0);

    // raise rail and travel
    robot->moveRail(0.40);

    if (to_the_left_of_the_samples) 
    {
        targetPosition = robot->getCurrentPosition();
        traj.clear();
        traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI)));
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleStatueBackup failed to complete");
    }

    servo->activatePump(false);
    servo->openValve();
    for (int i = 0; i < 3; i++)
        servo->openTube(i);

    if (to_the_left_of_the_samples)
    {
        targetPosition = robot->getCurrentPosition();
        traj = computeTrajectoryStraightLine(targetPosition, 250.0);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleStatueBackup failed to complete");
    } else 
    {
        targetPosition = robot->getCurrentPosition();
        traj = computeTrajectoryStraightLine(targetPosition, 300.0);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleStatueBackup failed to complete");
    }


    // 1 pt for each sample added in the zone
    robot->updateScore(3);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -100.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatueBackup failed to complete");    

    is_move_three_samples_finished = true;
    return(true);
}


bool Strategy::handleDigZone()
{
    if (is_handle_dig_zone_finished) 
    {
        return(true);
    }

    const int spacing_between_sites = 185;
    const int site_y = robotdimensions::CHASSIS_WIDTH + 80 + 60 -10;
    const int initial_first_x = 730 - 15 + 10;
    int first_site_x = initial_first_x;

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    // targetPosition.x -= 40 * std::cos(targetPosition.theta);
    // targetPosition.y -= 40 * std::sin(targetPosition.theta);
    // positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    targetPosition.x = 620;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    robot->setTrajectoryToFollow(traj);

    // ensure suctions are folded to reduce deloyed perimeter
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);

    MOVE_OR_ABORT("handleDigZone failed to complete");

    std::cout << "First correction" << std::endl;
    // Check with range sensor: are we at the right place ?
    double const MAXIMUM_RANGE_CORRECTION = 50;

    robot->wait(0.05);
    double measured_y = robot->getRangeSensorMeasurement(robot->isPlayingRightSide());
    targetPosition = robot->getCurrentPosition();
    // tamper with correction to correct half of the error
    measured_y = measured_y + (targetPosition.y - measured_y) / 2.0;
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

    traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleDigZone failed to complete");

    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);

    // Test all sites.

    // Test the first 3 sites
    ExcavationSquareColor color_detected = testExcavationSite();
    bool is_cross_detected = color_detected == ExcavationSquareColor::RED;
    int number_of_sites_pushed = 0;

    int number_of_tries = 1;

    while ((number_of_tries < 5) && color_detected == ExcavationSquareColor::NONE)
    {
        // go back 10 mm
        first_site_x = initial_first_x + number_of_tries * 10;
        
        // std::cout << "Retring x = " << static_cast<string>(first_site_x) << std::endl;

        traj = computeTrajectoryStraightLine(targetPosition, - 10);
        robot->setTrajectoryToFollow(traj);
        robot->waitForTrajectoryFinished();

        // test again
        color_detected = testExcavationSite();

        number_of_tries++;
    }

    if (color_detected == ExcavationSquareColor::NONE)
    {
        std::cout << "Still did not measure, resetting position" << std::endl;
        positions.clear();
        targetPosition = robot->getCurrentPosition();
        positions.push_back(targetPosition);
        targetPosition.x = initial_first_x;
        positions.push_back(targetPosition);
        robot->setTrajectoryToFollow(traj);
        robot->waitForTrajectoryFinished();
    }

    number_of_tries = 1;

    while ((number_of_tries < 5) && color_detected == ExcavationSquareColor::NONE)
    {
        // go back 10 mm
        first_site_x = initial_first_x - number_of_tries * 10;

        // std::cout << "Retring x = " << static_cast<string>(first_site_x) << std::endl;
        
        traj = computeTrajectoryStraightLine(targetPosition, 10);
        robot->setTrajectoryToFollow(traj);
        robot->waitForTrajectoryFinished();

        // test again
        color_detected = testExcavationSite();

        number_of_tries++;
    }

    if (color_detected == ExcavationSquareColor::NONE)
    {
        std::cout << "Still nothing, resetting position" << std::endl;
        first_site_x = initial_first_x;
    }

    if (shouldPushExcavationSite(color_detected))
    {
        number_of_sites_pushed++;
    }
    
    // test the 2 others
    for (int i = 1; i < 3; i++)
    {

        // if already pushed 2 sites, skip
        if (number_of_sites_pushed >= 2) 
        {
            break;
        }

        // Check range
        measured_y = robot->getRangeSensorMeasurement(robot->isPlayingRightSide());
        targetPosition = robot->getCurrentPosition();
        // tamper with correction to correct half of the error
        measured_y = measured_y + (targetPosition.y - measured_y) / 2.0;
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
        traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");



        // if cross is already detected, no need to measure
        if (is_cross_detected | (i == 1))
        {
            std::cout << "Site known to be pushed : no measurement required" << std::endl;
            
            // if sample is already pushed, do not push or count score
            if (samples_are_knocked_out[i]) 
            {
                std::cout << "Site is known to be already pushed" << std::endl;
            }
            else
            {
                pushExcavationSite();
                samples_are_knocked_out[i] = true;
            }

            number_of_sites_pushed++;

        } 
        else 
        {
            color_detected = testExcavationSite();
            is_cross_detected = color_detected == ExcavationSquareColor::RED;
            if (shouldPushExcavationSite(color_detected)) 
            {
                number_of_sites_pushed++;
                samples_are_knocked_out[i] = true;
            }
        }
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

        // Check range
        measured_y = robot->getRangeSensorMeasurement(robot->isPlayingRightSide());
        targetPosition = robot->getCurrentPosition();
        // tamper with correction to correct half of the error
        measured_y = measured_y + (targetPosition.y - measured_y) / 2.0;
        if (std::abs(sumOfAllCorrections + measured_y - targetPosition.y) < MAXIMUM_RANGE_CORRECTION)
        {
            std::cout << "Resetting position thanks to range sensor. Error:" << targetPosition.y - measured_y << std::endl;
            sumOfAllCorrections += measured_y - targetPosition.y;
            targetPosition.y = measured_y;
            robot->resetPosition(targetPosition, false, true, false);
        }
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = first_site_x + (targeted_site + 3 - 0.5) * spacing_between_sites;
        targetPosition.y = site_y;
        positions.push_back(targetPosition);
        targetPosition.x = first_site_x + (targeted_site + 3) * spacing_between_sites;
        targetPosition.y = site_y;
        positions.push_back(targetPosition);
        traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");

        if (know_targeted_site_is_ours)
        {
            std::cout << "Site known to be pushed : no measurement required" << std::endl;
            // bascule
            if (samples_are_knocked_out[targeted_site + 3]) {
                std::cout << "Site is known to be already pushed" << std::endl;
            }
            else
            {
                pushExcavationSite();
                samples_are_knocked_out[targeted_site + 3] = true;
            }

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
            ExcavationSquareColor color = testExcavationSite();

            // if measurement is successful then we know where our sites are
            if (color != ExcavationSquareColor::NONE) {
                // if we found our site
                if (shouldPushExcavationSite(color)) {
                    
                    // remember we pushed the sample
                    samples_are_knocked_out[targeted_site + 3] = true; 

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
    

    is_handle_dig_zone_finished = true;
    return(true);
}

bool Strategy::pushSamplesBelowShelter() 
{    
    if (is_push_samples_below_shelter_finished) 
    {
        return(true);
    }

    RobotPosition targetPosition = robot->getCurrentPosition();
    std::vector<RobotPosition> positions;

    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 1400;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.x = 950;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    targetPosition.y = 450;
    positions.push_back(targetPosition);
    targetPosition.x = 350;
    targetPosition.y = 350;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    // Move the rail so it doesn't hit the fake statue
    robot->moveRail(0.9);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    robot->wait(0.5);
    servo->moveClaw(claw::SIDE);
    (void) robot->waitForTrajectoryFinished();

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -150);
    robot->setTrajectoryToFollow(traj);
    robot->waitForTrajectoryFinished();

    // place claws and push all the way
    servo->moveClaw(claw::PUSH_SAMPLE_SHELTER);
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 190);
    robot->setTrajectoryToFollow(traj);
    robot->waitForTrajectoryFinished();

    // // go back a little
    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, -50);
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();

    // // turn
    // targetPosition = robot->getCurrentPosition();
    // traj.clear();
    // traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta - M_PI * 25.0 / 180.0)));
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();

    // // push
    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, 50);
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();

    // // go back a little
    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, -50);
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();

    // // turn
    // targetPosition = robot->getCurrentPosition();
    // traj.clear();
    // traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI * 50.0 / 180.0)));
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();

    // // push
    // targetPosition = robot->getCurrentPosition();
    // traj = computeTrajectoryStraightLine(targetPosition, 50);
    // robot->setTrajectoryToFollow(traj);
    // robot->waitForTrajectoryFinished();


    robot->updateScore(1*5); // push samples below shelter

    // move back and fold arms
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -180);
    robot->setTrajectoryToFollow(traj);

    (void) robot->waitForTrajectoryFinished(); 

    servo->moveClaw(claw::FOLD);

    is_push_samples_below_shelter_finished = true;
    return(true);
}
