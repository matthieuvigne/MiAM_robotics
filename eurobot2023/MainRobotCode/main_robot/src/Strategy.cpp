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
#include "miam_utils/raspberry_pi/RaspberryPi.h"

#include <common/DH_transform.hpp>


using namespace miam::trajectory;
using miam::RobotPosition;
using namespace kinematics;

#define LEFT_ARM_FIRST_SERVO_ID 10
#define RIGHT_ARM_FIRST_SERVO_ID 20

#define USE_CAMERA 1

#define TEST_SQUARE_MOVE 0 // make a square on the table to test motors
#define ENABLE_DYNAMIC_ACTION_CHOOSING 0 // use the dynamic action choosing feature

namespace main_robot
{

std::ostream& operator<<(std::ostream &s, const ArmPosition &armPosition) 
{
    return s << "[" << armPosition.r_ << ", " << armPosition.theta_ << ", " << armPosition.z_ << "]";
}

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;

int const PUMP_RIGHT = 12;
int const PUMP_LEFT = 13;

int const VALVE_RIGHT = 24;
int const VALVE_LEFT = 26;

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

    // Arms
    for (int i = 0; i < 4; i++)
    {
        servo->setTargetPosition(10 + i, 2048);
        servo->setTargetPosition(20 + i, 2048);
    }

    // Change P gain of the first servos of each arm to prevent vibrations
    servo->setPGain(10, 20);
    servo->setPGain(20, 20);
    servo->setPGain(11, 20);
    servo->setPGain(21, 20);

    // Fold arm
    // FIXME: why do I need to send it many times ?
    servo->setTargetPosition(11, 3000);
    servo->setTargetPosition(11, 3000);
    servo->setTargetPosition(11, 3000);
    servo->setTargetPosition(21, 1000);
    servo->setTargetPosition(21, 1000);
    servo->setTargetPosition(21, 1000);

    // Setup pumps and valves
    RPi_setupGPIO(PUMP_RIGHT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(PUMP_RIGHT, false);
    RPi_setupGPIO(PUMP_LEFT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(PUMP_LEFT, false);
    RPi_setupGPIO(VALVE_RIGHT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(VALVE_RIGHT, false);
    RPi_setupGPIO(VALVE_LEFT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(VALVE_LEFT, false);
}

void Strategy::shutdown()
{
    // Turn off pumps, open valves
    RPi_writeGPIO(PUMP_RIGHT, false);
    RPi_writeGPIO(PUMP_LEFT, false);
    RPi_writeGPIO(VALVE_RIGHT, false);
    RPi_writeGPIO(VALVE_LEFT, false);
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


    // Faire un carré pour faire un test de déplacement

#if FAIRE_CARRE

    targetPosition = motionController->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.x -= 500;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        targetPosition, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();


    targetPosition = motionController->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.y -= 500;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        targetPosition, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

    targetPosition = motionController->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.x += 500;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        targetPosition, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();


    targetPosition = motionController->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.y += 500;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        targetPosition, endPosition);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();

    targetPosition = motionController->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.x -= 500;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot->getParameters().getTrajConf(),
        targetPosition, endPosition);
    traj.pop_back();
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();


    bool moveSuccess = motionController->waitForTrajectoryFinished();
    if (moveSuccess)
        robot->updateScore(20);
    return;
#endif

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

    // Arm positions ; r theta z
    ArmPosition left_arm_center_up(100, 0, 250);
    ArmPosition left_arm_left_down(100, M_PI_4, 10);
    ArmPosition right_arm_center_up(100, 0, 250);
    ArmPosition right_arm_right_down(100, -M_PI_4, 10);

    // Get the initial position of the robot
    RobotPosition initial_position;
    initial_position.x = 2000 - robot->getParameters().CHASSIS_BACK;
    initial_position.y = 1725 + robot->getParameters().CHASSIS_WIDTH/2.0;
    initial_position.theta = M_PI;
    RobotPosition current_position = motionController->getCurrentPosition();


#if ENABLE_DYNAMIC_ACTION_CHOOSING

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
#else

    // Get the top right genoese and get it
    double distance = (genoese_top_left-current_position).norm();
    double coeff = (distance-cake_radius-robot_chassis_front)/distance;
    RobotPosition target_position = current_position + coeff*(genoese_top_right-current_position);
    go_to_straight_line(target_position);
    // set left arm to push genoese
    set_left_arm_position(left_arm_left_down);

    // Bring the first genoese to the top left cream and ganache
    target_position = cream_ganache_top_left + RobotPosition(150,0,0);
    go_to_straight_line(target_position);
    // reset arm positions
    set_left_arm_position(left_arm_center_up);
    set_left_arm_position(left_arm_center_up);
    target_position = cream_ganache_top_left + RobotPosition(robot_chassis_front,0,0);
    go_to_straight_line(target_position);

    // Build the cakes
    double constexpr RAD = M_PI/180.;
    target_position = target_position + 100*RobotPosition(-std::cos(45*RAD),std::sin(45*RAD),0);
    go_to_straight_line(target_position);

    // build cakes
    build_cakes();
    // set arms to push genoses
    set_left_arm_position(left_arm_left_down);
    set_right_arm_position(right_arm_right_down);

    // and then push them into the blue tray zone
    target_position = RobotPosition(target_position.x,2550-robot_chassis_front,0);
    go_to_straight_line(target_position);

    // reset arm positions
    set_left_arm_position(left_arm_center_up);
    set_right_arm_position(right_arm_center_up);

    // Go back 
    target_position = RobotPosition(target_position.x,target_position.y-300,M_PI);
    go_to_straight_line(target_position, true); // backward to not destroy cakes

    if(false)
    {
        // Option 1 -> bottom left genoese
        // -------------------------------

        // Reach the bottom left genoese (in a favorable position for the next action).
        RobotPosition tmp_position = cream_ganache_bottom_left + RobotPosition(250,0,0);
        distance = (genoese_bottom_left - tmp_position).norm();
        target_position = tmp_position + (distance+250)*(genoese_bottom_left - tmp_position)/distance;
        go_to_straight_line(target_position);

        // Push the bottom left genoese up to the bottom left cream and ganache
        // set left arm to push genoese
        set_left_arm_position(left_arm_left_down);

        target_position = tmp_position;
        go_to_straight_line(target_position);
        target_position = cream_ganache_bottom_left + RobotPosition(robot_chassis_front,0,0);
        go_to_straight_line(target_position);

        // reset arm positions
        set_left_arm_position(left_arm_center_up);

        // Build the cakes and push them into the closest blue plate zone
        target_position = target_position + 100*RobotPosition(-std::cos(45*RAD),std::sin(45*RAD),0);
        go_to_straight_line(target_position);

        // build cakes
        build_cakes();
        // set arms to push genoses
        set_left_arm_position(left_arm_left_down);
        set_right_arm_position(right_arm_right_down);

        target_position = RobotPosition(target_position.x,900-robot_chassis_front,0);
        go_to_straight_line(target_position);

        // Go to the final zone
        target_position = target_position + RobotPosition(0,-150,M_PI);
        go_to_straight_line(target_position, true); // backward to not destroy cakes
        target_position = RobotPosition(725,450,0);
        go_to_straight_line(target_position);
        target_position = RobotPosition(725,robot_chassis_front,0);
        go_to_straight_line(target_position);

    } else {

        // Option 2 -> bottom right genoese
        // --------------------------------

        // Reach the bottom left genoese (in a favorable position for the next action).
        RobotPosition tmp_position = cream_ganache_bottom_right + RobotPosition(-250,0,0);
        distance = (genoese_bottom_right - tmp_position).norm();
        target_position = tmp_position + (distance+250)*(genoese_bottom_right - tmp_position)/distance;
        go_to_straight_line(target_position);

        // Push the bottom left genoese up to the bottom left cream and ganache
        // set arms to push genoses
        set_left_arm_position(left_arm_left_down);
        target_position = tmp_position;
        go_to_straight_line(target_position);

        // left arm up and right arm down to push genose
        set_left_arm_position(left_arm_center_up);
        set_right_arm_position(right_arm_right_down);

        target_position = cream_ganache_bottom_right + RobotPosition(-robot_chassis_front,0,0);
        go_to_straight_line(target_position);

        // left arm up and right arm down to push genose
        set_left_arm_position(left_arm_left_down);
        set_right_arm_position(right_arm_center_up);

        // Build the cakes and push them into the closest blue plate zone
        target_position = target_position + 100*RobotPosition(std::cos(45*RAD),-std::sin(45*RAD),0);
        go_to_straight_line(target_position);

        // build cakes
        build_cakes();
        // set arms to push genoses
        set_left_arm_position(left_arm_left_down);
        set_right_arm_position(right_arm_right_down);

        target_position = RobotPosition(target_position.x,450+robot_chassis_front,0);
        go_to_straight_line(target_position);

        // Go to the final zone
        target_position = target_position + RobotPosition(0,150,M_PI);
        go_to_straight_line(target_position, true); // backward to not destroy cakes
        target_position = RobotPosition(725,450,0);
        go_to_straight_line(target_position);
        target_position = RobotPosition(725,robot_chassis_front,0);
        go_to_straight_line(target_position);

    }

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

        //~ if (goBackToDigSite())
        //~ {
            //~ robot->updateScore(20); //match completed
            //~ camera_.shutDown();
        //~ }
    }
}


std::vector<double> solve_arm_problem(ArmPosition armPosition)
{

    double r = armPosition.r_;
    double theta_rad = armPosition.theta_;
    double z = armPosition.z_;

    std::cout << ">>> Solving arm problem" << std::endl;

    // Initialize the robotical arm
    DHTransformVector arm = create_main_robot_arm();

    // Optimize the parameters to get the desired pose   
    Eigen::Vector3d pf;
    pf(0) = r * std::cos(theta_rad);
    pf(1) = r * std::sin(theta_rad);
    pf(2) = z;
    Eigen::Vector3d uf = Eigen::Vector3d::UnitZ();
    std::cout << "Desired coordinates (r, theta, z): " << armPosition << std::endl;
    std::cout << "Target position (x, y, theta) is: " << pf.transpose() << std::endl;
    std::cout << "Target x vector is: " << uf.transpose() << std::endl;
    
    // // Solve the optimization problem without constraints
    // OptimizationResult results = arm.optimize_position_x_direction(pf, uf);
    // if(!results.success) std::cout << "Problem failed... " << std::endl;
    // std::cout << "Problem 1 was solved with " << results.num_iters << " iterations." << std::endl;
    // std::cout << arm.print() << std::endl;
    

    std::vector<double> results_vector;
    results_vector.push_back(arm[0].get_parameter(Parameter::a2));    
    results_vector.push_back(arm[1].get_parameter(Parameter::a2));    
    results_vector.push_back(arm[2].get_parameter(Parameter::a2));    
    results_vector.push_back(arm[3].get_parameter(Parameter::a2));

    std::cout << results_vector[0] << " " << results_vector[1] << " " << results_vector[2] << " " << results_vector[3] << std::endl;

    return results_vector;
}

void Strategy::set_left_arm_position(ArmPosition armPosition)
{
    std::cout << "Moving left arm to: " << armPosition << std::endl;
    std::vector<double> results = solve_arm_problem(armPosition);

    // move servo
    for (int i = 0; i < 4; i++)
    {
        double angle = modulo(results[i]);
        servo->setTargetPosition(LEFT_ARM_FIRST_SERVO_ID + i, STS::radToServoValue(angle));
    }

    // TODO wait for servo movement
    robot->wait(1);
}

void Strategy::set_right_arm_position(ArmPosition armPosition)
{
    std::cout << "Moving right arm to: " << armPosition << std::endl;
    std::vector<double> results = solve_arm_problem(armPosition);

    // move servo
    for (int i = 0; i < 4; i++)
    {
        double angle = modulo(results[i]);
        servo->setTargetPosition(RIGHT_ARM_FIRST_SERVO_ID + i, STS::radToServoValue(angle));
    }

    // TODO wait for servo movement
    robot->wait(1);
}

void Strategy::build_cakes()
{
    std::cout << "Building cakes" << std::endl;

    // TODO wait to mimick arm movement
    robot->wait(5);
}

}
