/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "secondary_robot/Strategy.h"

#include <common/MotionPlanner.h>

using namespace miam::trajectory;
using miam::RobotPosition;


#define USE_CAMERA 1

#define TEST_SQUARE_MOVE 0 // make a square on the table to test motors
#define ENABLE_DYNAMIC_ACTION_CHOOSING 0 // use the dynamic action choosing feature

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;

namespace secondary_robot {

Strategy::Strategy()
{

}

void Strategy::setup(RobotInterface *robot)
{
    this->robot = robot;
    this->servo = robot->getServos();
    this->motionController = robot->getMotionController();

    // Set initial position
    RobotPosition targetPosition;
    targetPosition.x = 500;
    targetPosition.y = 200; ;
    targetPosition.theta = 0;
    motionController->resetPosition(targetPosition, true, true, true);
}

void Strategy::shutdown()
{
}

Action* Strategy::chooseNextAction(
    std::vector<Action>& actions,
    RobotPosition currentPosition,
    MotionPlanner motionPlanner
)
{

    Action* bestAction = &actions.front();
    double bestLoss = 0;

    int i = 0;

    for (auto & a : actions)
    {

        if (a.isActivated_)
        {
            // compute loss
            double timeItTakesToGoThere = motionPlanner.computeMotionTime(robot->getParameters().getTrajConf(), currentPosition, a.startPosition_);
            double currentLoss = a.nPointsToGain_ /(a.timeItTakes_ + timeItTakesToGoThere);

            std::cout << "Calcul pour " << i << " : " << timeItTakesToGoThere << ", " << currentLoss << std::endl;

            if (bestLoss < currentLoss)
            {
                bestAction = &a;
                bestLoss = currentLoss;
                std::cout << "Chosen action " << i << std::endl;
            }
        }
        i++;
    }

    return bestAction;
}

void Strategy::match_impl()
{

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    // create brain
    MotionPlanner motionPlanner(robot);


    double const robot_chassis_front = robot->getParameters().CHASSIS_FRONT;
    double const robot_chassis_back = robot->getParameters().CHASSIS_BACK;
    double const robot_half_chassis_width = robot->getParameters().CHASSIS_WIDTH/2.0;

    RobotPosition const left_of_cherry_distributor_bottom(700,200,0);
    RobotPosition const center_of_cherry_distributor_left(15,1500,0);
    RobotPosition const center_of_cherry_distributor_right(2000 - 15,1500,0);
    RobotPosition const center_of_cherry_distributor_bottom(1000,150,0);
    RobotPosition const center_of_cherry_distributor_top(1000,2850,0);

    double const distributor_width = 30;



#if ENABLE_DYNAMIC_ACTION_CHOOSING

    Action action1(100,1, RobotPosition(2000, 1700, 0));
    // Action action2(500, 1, RobotPosition(2200, 500, 0));
    // Action action3(15, 1, RobotPosition(1500, 200, 0));

    std::vector<Action> actionVector;
    actionVector.push_back(action1);
    // actionVector.push_back(action2);
    // actionVector.push_back(action3);



    while (!actionVector.empty())
    {

        std::cout << "Remaining actions : " << actionVector.size() << std::endl;
        for (auto v : actionVector)
        {
            std::cout << v << std::endl;
        }

        Action* nextAction = chooseNextAction(actionVector, motionController->getCurrentPosition(), motionPlanner);


        std::cout << "nextAction : " << *nextAction << std::endl;

        targetPosition = motionController->getCurrentPosition();
        traj = motionPlanner.computeTraj(robot->getParameters().getTrajConf(), targetPosition, nextAction->startPosition_);
        motionController->setTrajectoryToFollow(traj);
        bool moveSuccess = motionController->waitForTrajectoryFinished();

        if (!moveSuccess)
        {
            // attempt alternative route
            // assume object size is ~20 cm

            std::cout << "attempting alternative route" << std::endl;

            // attempt left
            RobotPosition left_point(motionController->getCurrentPosition());
            // 20 cm to the left
            left_point.x = left_point.x - 400 * sin(left_point.theta);
            left_point.y = left_point.y + 400 * cos(left_point.theta);

            if(left_point.x < table_dimensions::table_max_x and left_point.x > table_dimensions::table_min_x
                and left_point.y < table_dimensions::table_max_y and left_point.y > table_dimensions::table_min_y )
            {


                std::cout << "trying to go left" << std::endl;
                std::cout << "current position : " << motionController->getCurrentPosition() <<  std::endl;
                std::cout << "waypoint : " << left_point <<  std::endl;

                // // attempt following the trajectory
                // targetPosition = motionController->getCurrentPosition();
                // traj = computeTrajectoryStraightLineToPoint(targetPosition, left_point);
                // motionController->setTrajectoryToFollow(traj);

                targetPosition = motionController->getCurrentPosition();
                positions.clear();
                positions.push_back(targetPosition);
                positions.push_back(left_point);
                positions.push_back(nextAction->startPosition_);
                traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3);
                motionController->setTrajectoryToFollow(traj);

                if (motionController->waitForTrajectoryFinished())
                {
                    std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                    // targetPosition = motionController->getCurrentPosition();
                    // traj = motionPlanner.computeTraj(targetPosition, nextAction->startPosition_);
                    // motionController->setTrajectoryToFollow(traj);
                    // moveSuccess = motionController->waitForTrajectoryFinished();
                } else
                {
                    std::cout << "waypoint NOT reached :" << motionController->getCurrentPosition() <<  std::endl;

                }


            }

            if (!moveSuccess)
            {
                // attempt alternative route
                // assume object size is ~20 cm

                std::cout << "attempting alternative route" << std::endl;

                // attempt right
                RobotPosition right_point(motionController->getCurrentPosition());
                // 20 cm to the right
                right_point.x = right_point.x + 400 * sin(right_point.theta);
                right_point.y = right_point.y - 400 * cos(right_point.theta);

                if(right_point.x < table_dimensions::table_max_x and right_point.x > table_dimensions::table_min_x
                    and right_point.y < table_dimensions::table_max_y and right_point.y > table_dimensions::table_min_y )
                {


                    std::cout << "trying to go right" << std::endl;
                    std::cout << "current position : " << motionController->getCurrentPosition() <<  std::endl;
                    std::cout << "waypoint : " << right_point <<  std::endl;

                    // // attempt following the trajectory
                    // targetPosition = motionController->getCurrentPosition();
                    // traj = computeTrajectoryStraightLineToPoint(targetPosition, right_point);
                    // motionController->setTrajectoryToFollow(traj);

                    targetPosition = motionController->getCurrentPosition();
                    positions.clear();
                    positions.push_back(targetPosition);
                    positions.push_back(right_point);
                    positions.push_back(nextAction->startPosition_);
                    traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3);
                    motionController->setTrajectoryToFollow(traj);

                    if (motionController->waitForTrajectoryFinished())
                    {
                        std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                        // targetPosition = motionController->getCurrentPosition();
                        // traj = motionPlanner.computeTraj(targetPosition, nextAction->startPosition_);
                        // motionController->setTrajectoryToFollow(traj);
                        // moveSuccess = motionController->waitForTrajectoryFinished();
                    } else
                    {
                        std::cout << "waypoint NOT reached :" << motionController->getCurrentPosition() <<  std::endl;

                    }


                }
            }

        }


        if (moveSuccess)
        {
            nextAction->performAction(robot);
            actionVector.erase(std::find(actionVector.begin(), actionVector.end(), *nextAction));

            // If action successful, reactivate all actions
            for (auto & a : actionVector)
                nextAction->isActivated_ = true;
        }
        else
        {
            // penaliser l'action
            nextAction->isActivated_ = false;
        }



    }
#else

    // // grab cheries in front of robot
    // go_to_straight_line(left_of_cherry_distributor_bottom);

    // // set rail height and tilt
    // set_reservoir_tilt(ReservoirTilt::UP);
    // move_rail(RailHeight::CHERRY_DISTRIBUTOR);
    // set_brush_move(BrushDirection::TOWARDS_BACK);

    // // go front
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(100, 0, 0));
    // // wait
    // robot->wait(3);
    // // go back
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(-100, 0, 0), true); // backwards


    // code de sophie

    // Option 1 -> Bottom, right & top cherries
    // -------------------------------

    std::vector<RobotPosition> targetPositions;

    // Get the  bottom cheeries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_bottom - RobotPosition(robot_chassis_front + 151 + distributor_width / 2,0,0));
    targetPositions.push_back(center_of_cherry_distributor_bottom - RobotPosition(robot_chassis_front + 150 + distributor_width / 2,0,0));
    go_to_rounded_corner(targetPositions);

    // grab cherries
    grab_cherries();

    // Go to get right cherries then get the top cherries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 700,0,0));
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 151 + distributor_width / 2,0,0));
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 150 + distributor_width / 2,0,0));
    go_to_rounded_corner(targetPositions);

    grab_cherries();


    // avoid the line 
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(RobotPosition(500, 3000-600,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front + 155 + distributor_width / 2, 0,0));
    go_to_rounded_corner(targetPositions, true);

    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front + 151 + distributor_width / 2, 0,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front + 150 + distributor_width / 2, 0,0));
    go_to_rounded_corner(targetPositions);

    grab_cherries();
      
    // Go to put the cherries in the basket
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(RobotPosition(225,3000 - robot_chassis_front - 151, 0));
    targetPositions.push_back(RobotPosition(225,3000 - robot_chassis_front - 150,M_PI_2));
    go_to_rounded_corner(targetPositions);

    put_cherries_in_the_basket();

    // Go to the final zone
    go_to_straight_line(RobotPosition(725,200,0), true);
    // go_to_straight_line(RobotPosition(300,1300,0), true);



#endif

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = stratMain.native_handle();
    createdThreads_.push_back(handle);
    stratMain.detach();

    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    usleep(50000);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
    }
}

void Strategy::set_brush_move(BrushDirection brushDirection)
{
    if (brushDirection == BrushDirection::OFF)
    {
        // set motor speed to 0
        // TODO
    }
    else if (brushDirection == BrushDirection::TOWARDS_BACK) 
    {
        // set motor speed to +x
        // TODO
    }
    else if (brushDirection == BrushDirection::TOWARDS_BACK) 
    {
        // set motor speed to +x
        // TODO
    }
}

void Strategy::set_reservoir_tilt(ReservoirTilt reservoirTilt)
{
    if (reservoirTilt == ReservoirTilt::DOWN)
    {
        // set servo 
        // TODO
    }
    else if (reservoirTilt == ReservoirTilt::UP)
    {
        // set servo 
        // TODO
    }
}

void Strategy::grab_cherries()
{
    // set rail height and tilt
    set_reservoir_tilt(ReservoirTilt::UP);
    move_rail(RailHeight::CHERRY_DISTRIBUTOR);
    set_brush_move(BrushDirection::TOWARDS_BACK);
    // go front
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(150, 0, 0));
    go_forward(150);

    // wait
    robot->wait(3);
    // go back
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(-150, 0, 0), true); // backwards
    go_forward(-150);
    move_rail(RailHeight::MIDDLE);
}

void Strategy::put_cherries_in_the_basket()
{
    // put rail in the right height
    move_rail(RailHeight::CHERRY_BASKET);

    // go front
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(150, 0, 0));
    go_forward(150);

    // tilt and push cherries
    set_reservoir_tilt(ReservoirTilt::DOWN);
    set_brush_move(BrushDirection::TOWARDS_FRONT);
    // wait
    robot->wait(3);
    // prepare to go back
    set_reservoir_tilt(ReservoirTilt::UP);
    set_brush_move(BrushDirection::OFF);

    // go back
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(-150, 0, 0), true); // backwards
    go_forward(-150);

    move_rail(RailHeight::MIDDLE);
}

void Strategy::move_rail(RailHeight railHeight)
{
    std::cout << "Moving rail to: " << railHeight << std::endl;
    robot->moveRail((int) railHeight);
}

}
