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
    targetPosition = motionController->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 925;
    targetPosition.y = 620;
    positions.push_back(targetPosition);
    targetPosition.x = 930;
    targetPosition.y = 620;
    positions.push_back(targetPosition);
    TrajectoryVector traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
    motionController->setTrajectoryToFollow(traj);
    // servo->openValve() ;
    // servo->openTube(0);
    // servo->openTube(1);
    // servo->openTube(2);

    // servo->moveArm(true, arm::FOLD);
    // servo->moveFinger(true, finger::FOLD);
    // servo->moveArm(false, arm::FOLD);
    // servo->moveFinger(false, finger::FOLD);

    return motionController->waitForTrajectoryFinished();
}


Strategy::Strategy()
{
    // is_handle_statue_finished = false;
    // is_move_side_sample_finished = false;
    // is_handle_side_triple_samples_finished = false;
    // is_move_three_samples_finished = false;
    // is_handle_dig_zone_finished = false;
    // is_bonus_already_counted = false;
    // is_push_samples_below_shelter_finished = false;

}

void Strategy::setup(RobotInterface *robot)
{
    this->robot = robot;
    // this->servo = robot->getServos();
    this->motionController = robot->getMotionController();

    // servo->moveStatue(statue::FOLD);
    // servo->activateMagnet(false);

    // dropElements();

    // servo->moveArm(true, arm::FOLD);
    // servo->moveFinger(true, finger::FOLD);
    // servo->moveArm(false, arm::FOLD);
    // servo->moveFinger(false, finger::FOLD);
    // servo->moveClaw(claw::FOLD);

    // //init ventouse & rail
    // for (int i = 0; i < 3; i++)
    //     servo->moveSuction(i, suction::FOLD);
    // servo->moveSuction(1, suction::FOLD);

    // if (robot->getTestMode())
    // {
    //     robot->moveRail(0.5);
    //     robot->wait(2.0);
    //     robot->moveRail(0.65);
    // }
    // Set initial position
    RobotPosition targetPosition;
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    motionController->resetPosition(targetPosition, true, true, true);
}


Action* Strategy::chooseNextAction(
    std::vector<Action>& actions,
    RobotPosition currentPosition,
    MotionPlanning motionPlanner
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
            double timeItTakesToGoThere = motionPlanner.computeMotionTime(currentPosition, a.startPosition_);
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
    robot->updateScore(2);  //depose statuette
    robot->updateScore(2);  //depose vitrine

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    motionController->resetPosition(targetPosition, true, true, true);
    robot->wait(0.05);

    endPosition = motionController->getCurrentPosition();
    endPosition.x += 1000;

    traj = computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
    motionController->setTrajectoryToFollow(traj);

    motionController->waitForTrajectoryFinished();

    return;


    // create brain
    MotionPlanning motion_planner;

    Action action1(100,1, RobotPosition(1500, 1200, 0));
    Action action2(50, 1, RobotPosition(1500, 1800, 0));
    Action action3(15, 1, RobotPosition(500, 2200, 0));

    std::vector<Action> actionVector;
    actionVector.push_back(action1);
    actionVector.push_back(action2);
    actionVector.push_back(action3);



    while (!actionVector.empty())
    {

        std::cout << "Remaining actions : " << actionVector.size() << std::endl;
        for (auto v : actionVector)
        {
            std::cout << v << std::endl;
        }

        Action* nextAction = chooseNextAction(actionVector, motionController->getCurrentPosition(), motion_planner);


        std::cout << "nextAction : " << *nextAction << std::endl;

        targetPosition = motionController->getCurrentPosition();
        traj = motion_planner.computeTraj(targetPosition, nextAction->startPosition_);
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
                traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
                motionController->setTrajectoryToFollow(traj);

                if (motionController->waitForTrajectoryFinished())
                {
                    std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                    // targetPosition = motionController->getCurrentPosition();
                    // traj = motion_planner.computeTraj(targetPosition, nextAction->startPosition_);
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
                    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.3);
                    motionController->setTrajectoryToFollow(traj);

                    if (motionController->waitForTrajectoryFinished())
                    {
                        std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                        // targetPosition = motionController->getCurrentPosition();
                        // traj = motion_planner.computeTraj(targetPosition, nextAction->startPosition_);
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



    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    // Update config.
    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = stratMain.native_handle();
    createdThreads_.push_back(handle);
    stratMain.detach();

    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    usleep(50000);
    // servo->moveRail(robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;

        // servo->activatePump(false);

        if (goBackToDigSite())
        {
            robot->updateScore(20); //match completed
            camera_.shutDown();
        }
    }
}

