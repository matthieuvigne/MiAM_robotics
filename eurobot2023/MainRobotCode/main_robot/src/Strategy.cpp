/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "main_robot/Strategy.h"

using namespace miam::trajectory;
using miam::RobotPosition;


#define USE_CAMERA 1

namespace main_robot
{

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;


Strategy::Strategy()
{
  // [TODO]
}

void Strategy::setup(RobotInterface *robot)
{
    // Get robot
    this->robot = robot;
    this->servo = robot->getServos();
    this->motionController = robot->getMotionController();

    // Set initial position
    RobotPosition targetPosition;
    targetPosition.x = 2000 - robot->getParameters().CHASSIS_BACK;
    targetPosition.y = 1725 + robot->getParameters().CHASSIS_WIDTH/2.0;
    targetPosition.theta = M_PI;
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

    // if all actions are deactivated, reactivate all actions
    bool all_actions_deactivated = true;
    for (auto & a : actions)
    {
        if (a.isActivated_)
            all_actions_deactivated = false;
    }
    if (all_actions_deactivated)
    {
        std::cout << "All actions are deactivated: reactivate all" << std::endl;
        for (auto & a : actions)
            a.isActivated_ = true;
    }

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

    // Create brain
    MotionPlanning motion_planner;
    std::vector<Action> actionVector;
    
    // Common cake dimensions
    double const cake_radius = 60; // [mm]
    double const robot_chassis_front = robot->getParameters().CHASSIS_FRONT;
    
    // Initial positions of the genoeses
    RobotPosition const genoese_top_left(725,1875,0);
    RobotPosition const genoese_top_right(1275,1875,0);
    RobotPosition const genoese_bottom_left(725,1125,0);
    RobotPosition const genoese_bottom_right(1275,1125,0);
    
    // Initial barycenters of the cream/ganache couples
    RobotPosition const cream_ganache_top_left(225,2325,0);
    RobotPosition const cream_ganache_top_right(1775,2325,0);
    RobotPosition const cream_ganache_bottom_left(225,675,0);
    RobotPosition const cream_ganache_bottom_right(1775,675,0);
    
    // Get the initial position of the robot
    RobotPosition initial_position;
    initial_position.x = 2000 - robot->getParameters().CHASSIS_BACK;
    initial_position.y = 1725 + robot->getParameters().CHASSIS_WIDTH/2.0;
    initial_position.theta = M_PI;
    RobotPosition current_position = motionController->getCurrentPosition();
    
    // Get the top right genoese and get it
    double distance = (genoese_top_left-current_position).norm();
    double coeff = (distance-cake_radius-robot_chassis_front)/distance;
    RobotPosition target_position = current_position + coeff*(genoese_top_right-current_position);
    actionVector.push_back(Action(0, 1, target_position));
    
    // Bring the first genoese to the top left cream and ganache
    target_position = cream_ganache_top_left + RobotPosition(150,0,0);
    actionVector.push_back(Action(0, 1, target_position));
    target_position = cream_ganache_top_left + RobotPosition(robot_chassis_front,0,0);
    actionVector.push_back(Action(0, 1, target_position));
    
    // Build the cakes and then push them into the blue tray zone
    double constexpr RAD = M_PI/180.;
    target_position = target_position + 100*RobotPosition(-std::cos(45*RAD),std::sin(45*RAD),0);
    actionVector.push_back(Action(0, 1, target_position));
    target_position = RobotPosition(target_position.x,2550-robot_chassis_front,0);
    actionVector.push_back(Action(0, 1, target_position));
       
    if(true)
    {
      // Option 1 -> bottom left genoese
      // -------------------------------
      
      // Go back and reach the bottom left genoese (in a favorable position for the next action).
      target_position = RobotPosition(target_position.x,target_position.y-300,M_PI);
      actionVector.push_back(Action(0, 1, target_position));
      RobotPosition tmp_position = cream_ganache_bottom_left + RobotPosition(250,0,0);
      distance = (genoese_bottom_left - tmp_position).norm();
      target_position = tmp_position + (distance+250)*(genoese_bottom_left - tmp_position)/distance;
      actionVector.push_back(Action(0, 1, target_position));
      
      // Push the bottom left genoese up to the bottom left cream and ganache
      target_position = tmp_position;
      actionVector.push_back(Action(0, 1, target_position));
      target_position = cream_ganache_bottom_left + RobotPosition(robot_chassis_front,0,0);
      actionVector.push_back(Action(0, 1, target_position));

      // Build the cakes and push them into the closest blue plate zone
      target_position = target_position + 100*RobotPosition(-std::cos(45*RAD),std::sin(45*RAD),0);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(target_position.x,900-robot_chassis_front,0);
      actionVector.push_back(Action(0, 1, target_position));

      // Go to the final zone
      target_position = target_position + RobotPosition(0,-150,M_PI);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(725,450,0);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(725,robot_chassis_front,0);
      actionVector.push_back(Action(0, 1, target_position));
      
    } else {
      
      // Option 2 -> bottom right genoese
      // --------------------------------
      
      // Go back and reach the bottom left genoese (in a favorable position for the next action).
      target_position = RobotPosition(target_position.x,target_position.y-300,M_PI);
      actionVector.push_back(Action(0, 1, target_position));
      RobotPosition tmp_position = cream_ganache_bottom_right + RobotPosition(-250,0,0);
      distance = (genoese_bottom_right - tmp_position).norm();
      target_position = tmp_position + (distance+250)*(genoese_bottom_right - tmp_position)/distance;
      actionVector.push_back(Action(0, 1, target_position));
      
      // Push the bottom left genoese up to the bottom left cream and ganache
      target_position = tmp_position;
      actionVector.push_back(Action(0, 1, target_position));
      target_position = cream_ganache_bottom_right + RobotPosition(-robot_chassis_front,0,0);
      actionVector.push_back(Action(0, 1, target_position));

      // Build the cakes and push them into the closest blue plate zone
      target_position = target_position + 100*RobotPosition(std::cos(45*RAD),-std::sin(45*RAD),0);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(target_position.x,450+robot_chassis_front,0);
      actionVector.push_back(Action(0, 1, target_position));

      // Go to the final zone
      target_position = target_position + RobotPosition(0,150,M_PI);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(725,450,0);
      actionVector.push_back(Action(0, 1, target_position));
      target_position = RobotPosition(725,robot_chassis_front,0);
      actionVector.push_back(Action(0, 1, target_position));
      
    }

    while (!actionVector.empty())
    {

        std::cout << ">>> RE-ASSESSING ACTIONS" << std::endl;

        std::cout << "Remaining actions : " << actionVector.size() << std::endl;
        for (auto v : actionVector)
        {
            std::cout << v << std::endl;
        }

        Action* nextAction = chooseNextAction(actionVector, motionController->getCurrentPosition(), motion_planner);
        std::cout << "nextAction : " << *nextAction << std::endl;

        targetPosition = motionController->getCurrentPosition();
        //~ traj = motion_planner.computeTraj(robot->getParameters().getTrajConf(), targetPosition, nextAction->startPosition_);
        // Petite couille qu'il faudra enlever (ici pour gerer le mouvement arriere).
        traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
          targetPosition, nextAction->startPosition_,0.0,nextAction->startPosition_.theta==M_PI);
        // Disable avoidance?
        // for (auto i : traj)
        // {
        //     i->setAvoidanceEnabled(false);
        // }
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

            // left point plus 20 cm
            RobotPosition left_point_further(left_point);
            left_point_further.x += 400 * cos(left_point.theta);
            left_point_further.y += 400 * sin(left_point.theta);

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
                positions.push_back(left_point_further);
                positions.push_back(nextAction->startPosition_);
                // Seconde petite couille pour gerer le mouvement 'backward'
                traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3,nextAction->startPosition_.theta==M_PI);
                motionController->setTrajectoryToFollow(traj);

                moveSuccess = motionController->waitForTrajectoryFinished();

                if (moveSuccess)
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
                    traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3);
                    motionController->setTrajectoryToFollow(traj);

                    moveSuccess = motionController->waitForTrajectoryFinished();

                    if (moveSuccess)
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

            // Reactivate all actions
            for (auto & a : actionVector)
                a.isActivated_ = true;
        }
        else
        {
            // Penaliser l'action
            nextAction->isActivated_ = false;
        }

    }



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

        servo->activatePump(false);

        //~ if (goBackToDigSite())
        //~ {
            //~ robot->updateScore(20); //match completed
            //~ camera_.shutDown();
        //~ }
    }
}


}