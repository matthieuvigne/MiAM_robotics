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

// Handle the statue: grab it, swap, and drop the real statue.
// Abort as soon as one action fails.
void Strategy::handleStatue()
{
    //Go back
    RobotPosition targetPosition = robot->getCurrentPosition();
    std::vector<RobotPosition> positions;
    positions.push_back(targetPosition);
    targetPosition.x = 250 + robotdimensions::CHASSIS_BACK;
    targetPosition.y = 250 + robotdimensions::CHASSIS_BACK;
    positions.push_back(targetPosition);
    targetPosition.x = 225 + robotdimensions::CHASSIS_BACK;
    targetPosition.y = 225 + robotdimensions::CHASSIS_BACK;
    positions.push_back(targetPosition);

    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200, 0.3, true);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveStatue(statue::CATCH);
    servo->activateMagnet(true);
    robot->wait(0.6);
    std::cout << robot->getCurrentPosition() << std::endl;
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(5);

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
    MOVE_OR_ABORT("handleStatue failed to complete");

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, 20.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    // Place fake statue
    robot->moveRail(0.3);
    servo->moveSuction(1, suction::DROP_STATUE);
    dropElements();
    robot->wait(0.2);
    robot->updateScore(10);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -30.0);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

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
    targetPosition.y = 2000 - robotdimensions::CHASSIS_BACK - 5;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 300.0, 0.3, true);

    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleStatue failed to complete");

    servo->moveStatue(statue::DROP);
    robot->wait(0.1);
    servo->activateMagnet(false);
    servo->moveStatue(statue::TRANSPORT);
    robot->updateScore(20);
}

void Strategy::moveSideSample()
{
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1680 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 30;
    positions.push_back(targetPosition);
    TrajectoryVector traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 300.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    usleep(1000000);
    servo->moveStatue(statue::FOLD);
    MOVE_OR_ABORT("moveSideSample failed to complete");

    servo->closeTube(0);
    servo->closeTube(2);
    servo->activatePump(true);
    servo->closeValve();
    robot->moveRail(0.58);
    robot->wait(1.0);
    robot->moveRail(0.8);

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
}

void Strategy::handleSideTripleSamples()
{
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_BACK + 270;
    targetPosition.y = 750;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 190;
    targetPosition.y = 750;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    robot->moveRail(0.25);
    servo->moveSuction(1, suction::LOWER_SAMPLE);
    MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

    // Grab and drop the three samples
    for (int i = 0; i < 3; i++)
    {
        servo->closeValve();
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
        robot->wait(1.0);
        robot->moveRail(0.6);
        targetPosition = robot->getCurrentPosition();
        traj = computeTrajectoryStraightLine(targetPosition, -60);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

        targetPosition = robot->getCurrentPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.y += 70;
        positions.push_back(targetPosition);
        traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");

        dropElements();
        robot->wait(0.3);
        servo->moveSuction(1, suction::LOWER_SAMPLE);
        targetPosition = robot->getCurrentPosition();
        robot->setTrajectoryToFollow(computeTrajectoryStraightLine(targetPosition, -70));
        MOVE_OR_ABORT("handleSideTripleSamples failed to complete");
    }
}

void Strategy::moveThreeSamples()
{
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
    traj = computeTrajectoryStraightLine(targetPosition, 60.0);
    robot->setTrajectoryToFollow(traj);
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
    targetPosition.y = 2000 - robotdimensions::SUCTION_CENTER - 105;
    targetPosition.theta = M_PI_2;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 400.0, 0.3);
    robot->setTrajectoryToFollow(traj);
    robot->wait(1.0);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::LOWER_SAMPLE);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");

    dropElements();
    robot->moveRail(0.35);
    robot->wait(0.5);
    robot->updateScore(9);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition, -90);
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("moveThreeSamples failed to complete");
}

void Strategy::handleDigZone()
{
    const int spacing_between_sites = 185;
    const int site_y = robotdimensions::CHASSIS_WIDTH + 80 + 60 -10;
    const int first_site_x = 730 - 15;

    std::vector<RobotPosition> positions;
    RobotPosition targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x -= 40 * std::cos(targetPosition.theta);
    targetPosition.y -= 40 * std::sin(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    targetPosition.x = 620;
    targetPosition.y = site_y;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.1, true);
    robot->setTrajectoryToFollow(traj);
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
    robot->setTrajectoryToFollow(traj);
    MOVE_OR_ABORT("handleDigZone failed to complete");

    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);

    // Test all sites.
    testExcavationSite();
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
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");
        testExcavationSite();
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
        traj = computeTrajectoryRoundedCorner(positions, 100.0, 0.3, true);
        robot->setTrajectoryToFollow(traj);
        MOVE_OR_ABORT("handleDigZone failed to complete");

        if (know_targeted_site_is_ours)
        {
            // bascule
            pushExcavationSite();
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
            ExcavationSquareColor color = testExcavationSite();

            // if measurement is successful then we know where our sites are
            if (color != ExcavationSquareColor::NONE) {
                // if we found our site
                if (shouldPushExcavationSite(color)) {
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
