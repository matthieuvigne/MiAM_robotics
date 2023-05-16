/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/trajectory/PathPlanner.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include "main_robot/Strategy.h"
#include "common/DH_transform.hpp"
#include "common/MotionPlanner.h"
#include "common/ArmInverseKinematics.hpp"


using namespace miam::trajectory;
using miam::RobotPosition;
using namespace kinematics;

#define RIGHT_ARM_FIRST_SERVO_ID 10
#define LEFT_ARM_FIRST_SERVO_ID 20

#define FUNNY_ACTION_LEFT_SERVO_ID 6
#define FUNNY_ACTION_RIGHT_SERVO_ID 7

#define TEST_CAKE 1
#define USE_CAMERA 1
#define ENABLE_DYNAMIC_ACTION_CHOOSING 0 // use the dynamic action choosing feature
#define TEST_MPC_PLANNER 0

namespace main_robot
{

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;
RobotPosition START_POSITION(0, 1125, 0);


//--------------------------------------------------------------------------------------------------

Strategy::Strategy()
{
    // Empty on purpose
}

//--------------------------------------------------------------------------------------------------

bool Strategy::setup(RobotInterface *robot)
{
    std::cout << "Begin the setup" << std::endl;

    // Get robot
    this->robot = robot;
    this->servo = robot->getServos();
    this->motionController = robot->getMotionController();


    // Setup pumps and valves
    RPi_setupGPIO(PUMP_RIGHT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(PUMP_RIGHT, false);
    RPi_setupGPIO(PUMP_LEFT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(PUMP_LEFT, false);
    RPi_setupGPIO(VALVE_RIGHT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(VALVE_RIGHT, false);
    RPi_setupGPIO(VALVE_LEFT, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(VALVE_LEFT, false);

    // Set initial position: bottom left
    START_POSITION.x = robot->getParameters().CHASSIS_BACK;
    motionController->resetPosition(START_POSITION, true, true, true);



    // Change P gain of the first servos of each arm to prevent vibrations
    servo->setPIDGains(RIGHT_ARM, 25, 15, 0);
    servo->setPIDGains(LEFT_ARM, 25, 15, 0);
    servo->setPIDGains(RIGHT_ARM + 1, 20, 15, 0);
    servo->setPIDGains(LEFT_ARM + 1, 20, 1, 0);

#if TEST_CAKE
    // Fold arm (to comment)
    servo->setTargetPosition(FUNNY_ACTION_LEFT_SERVO_ID, 2048);
    servo->setTargetPosition(FUNNY_ACTION_RIGHT_SERVO_ID, 2048);
    servo->setTargetPosition(RIGHT_ARM, STS::radToServoValue(M_PI_2));
    servo->setTargetPosition(LEFT_ARM, STS::radToServoValue(-M_PI_2));
    for (int i = 1; i < 4; i++)
    {
        servo->setTargetPosition(RIGHT_ARM + i, 2048);
        servo->setTargetPosition(LEFT_ARM + i, 2048);
    }
    servo->setTargetPosition(RIGHT_ARM + 1, STS::radToServoValue(M_PI_2));
    servo->setTargetPosition(LEFT_ARM + 1, STS::radToServoValue(-M_PI_2));
    setTargetPosition(LEFT_ARM, ABS, 0.15, ABS, 80*arm::RAD, ABS, arm::PILE_CLEAR_HEIGHT);
    setTargetPosition(RIGHT_ARM, ABS, 0.15, ABS, 80*arm::RAD, ABS, arm::PILE_CLEAR_HEIGHT);
    runActionBlock();
#else
    for (int i = 0; i < 4; i++)
    {
        servo->setTargetPosition(RIGHT_ARM + i, 2048);
        servo->setTargetPosition(LEFT_ARM + i, 2048);
    }
    servo->setTargetPosition(RIGHT_ARM + 1, STS::radToServoValue(M_PI_2));
    servo->setTargetPosition(LEFT_ARM + 1, STS::radToServoValue(-M_PI_2));
#endif

    std::cout << "End of the setup" << std::endl;
    return true;
}

void Strategy::funnyAction()
{
    servo->setTargetPosition(FUNNY_ACTION_LEFT_SERVO_ID, 1500);
    servo->setTargetPosition(FUNNY_ACTION_RIGHT_SERVO_ID, 2500);
}


//--------------------------------------------------------------------------------------------------

void Strategy::periodicAction()
{

}

//--------------------------------------------------------------------------------------------------

void Strategy::shutdown()
{
    // Turn off pumps, open valves
    RPi_writeGPIO(PUMP_RIGHT, false);
    RPi_writeGPIO(PUMP_LEFT, false);
    RPi_writeGPIO(VALVE_RIGHT, false);
    RPi_writeGPIO(VALVE_LEFT, false);
}

//--------------------------------------------------------------------------------------------------

Action* Strategy::chooseNextAction(
    std::vector<Action>& actions,
    RobotPosition currentPosition,
    MotionPlanner& motionPlanner
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

//--------------------------------------------------------------------------------------------------

namespace arm_positions{
    double const GROUND_HEIGHT = -0.195;

    // Position of the cakes
    double const CAKES_FRONT_DISTANCE = 0.115;
    double const CAKES_SIDE_DISTANCE = 0.100;
    double const FRONT_RIGHT_ANGLE = -0.27;
    double const FRONT_LEFT_ANGLE = 0.27;
    double const SIDE_ANGLE = 1.2;

    double const PILE_CLEAR_HEIGHT = GROUND_HEIGHT + 0.085;
    // double const PILE_CLEAR_HEIGHT = GROUND_HEIGHT + 0.085;

    ArmPosition DISTRIBUTOR_CHERRY(125, -0.55, -130);

}
//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
    // Initialize arm positions
    ArmPosition sidePile(
      arm_positions::CAKES_SIDE_DISTANCE,
      arm_positions::SIDE_ANGLE,
      arm_positions::GROUND_HEIGHT + 0.070);
    setArmPosition(RIGHT_ARM_FIRST_SERVO_ID, sidePile);
    setArmPosition(LEFT_ARM_FIRST_SERVO_ID, sidePile);

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    // Create brain
    MotionPlanner* motionPlanner = motionController->motionPlanner_;
    std::vector<Action> actionVector;

    RobotParameters const robotParameters = robot->getParameters();

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
    RobotPosition const cream_ganache_bottom_left(225,675+30,0);
    RobotPosition const cream_ganache_bottom_right(1775,675+30,0);

    // Arm positions ; r theta z
    ArmPosition left_arm_center_up(100, 0, 250);
    ArmPosition left_arm_left_down(100, M_PI_4, 10);
    ArmPosition right_arm_center_up(100, 0, 250);
    ArmPosition right_arm_right_down(100, -M_PI_4, 10);

    RobotPosition tmp_position;
    RobotPosition current_position = motionController->getCurrentPosition();

    // Initialization -> open arms
    //~ servo->setTargetPosition(RIGHT_ARM, STS::radToServoValue(0.5));
    //~ servo->setTargetPosition(LEFT_ARM, STS::radToServoValue(-0.5));

    // Grab first genoise with the arms aside
    #if 0
    clearActionSequence();
    targetPosition = genoese_bottom_left - RobotPosition(robotParameters.CHASSIS_FRONT + 60, 0, 0);
    go_to_straight_line(targetPosition);
    setTargetPosition(LEFT_ARM, ABS, 0.15, ABS, 40*arm::RAD, ABS, arm::PILE_CLEAR_HEIGHT);
    setTargetPosition(LEFT_ARM, REL, 0.00, REL, 0, ABS, arm::GROUND_HEIGHT + 1e-2);
    setTargetPosition(LEFT_ARM, REL, 0.00, REL, -30*arm::RAD, REL, 0);
    setTargetPosition(RIGHT_ARM, ABS, 0.15, ABS, 40*arm::RAD, ABS, arm::PILE_CLEAR_HEIGHT);
    setTargetPosition(RIGHT_ARM, REL, 0.00, REL, 0, ABS, arm::GROUND_HEIGHT + 1e-2);
    setTargetPosition(RIGHT_ARM, REL, 0.00, REL, -30*arm::RAD, REL, 0);
    runActionBlock();
    robot->wait(0.25);

    // Go between the next two cakes, stop just before them.
    targetPosition = cream_ganache_bottom_right + RobotPosition(-600,0,0);
    go_to_straight_line(targetPosition);
    robot->wait(0.25);
    targetPosition = cream_ganache_bottom_right + RobotPosition(-300,0,0);
    go_to_straight_line(targetPosition);
    turn_around_point(5*arm::RAD);
    motionController->resetPosition(targetPosition,true,true,true);
    robot->wait(0.25);
    setTargetPosition(LEFT_ARM, REL, 0.00, ABS, 70*arm::RAD, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0.00, ABS, 70*arm::RAD, REL, 0);
    runActionBlock();
    targetPosition = cream_ganache_bottom_right - RobotPosition(robotParameters.CHASSIS_FRONT+50,0,0);
    go_to_straight_line(targetPosition);
    robot->wait(0.25);
    targetPosition.x = 2000 - cake_radius - robotParameters.CHASSIS_FRONT;
    go_to_straight_line(targetPosition);
    targetPosition.x = targetPosition.x - 100;
    go_to_straight_line(targetPosition, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
    runActionBlock();
    robot->wait(1.00);
    targetPosition.x += 30;
    go_to_straight_line(targetPosition);
    #endif

    //~ // Funny action
    //~ for (int i = 0; i < 4; i++)
    //~ {
        //~ servo->setTargetPosition(RIGHT_ARM + i, 2048);
        //~ servo->setTargetPosition(LEFT_ARM + i, 2048);
    //~ }
    //~ servo->setTargetPosition(RIGHT_ARM , STS::radToServoValue(-25*arm::RAD));
    //~ servo->setTargetPosition(RIGHT_ARM + 1, STS::radToServoValue(M_PI_2));
    //~ servo->setTargetPosition(LEFT_ARM, STS::radToServoValue(25*arm::RAD));
    //~ servo->setTargetPosition(LEFT_ARM + 1, STS::radToServoValue(-M_PI_2));
    //~ RPi_setupGPIO(25, PiGPIOMode::PI_GPIO_OUTPUT);
    //~ funnyAction();
    //~ while(true);;

    // Build the cakes
    buildCakes();
    while(true);;

    //~ targetPosition = targetPosition + RobotPosition(cake_radius,0,0);
    //~ // Go back, set arms and fuck the cakes
    //~ targetPosition = targetPosition + RobotPosition(-200,0,0);
    //~ go_to_straight_line(targetPosition, true);
    //~ setTargetPosition(LEFT_ARM, REL, 0, ABS, 10*arm::RAD, REL, 0);
    //~ setTargetPosition(RIGHT_ARM, REL, 0, ABS, 10*arm::RAD, REL, 0);
    //~ runActionBlock();
    //~ targetPosition = targetPosition + RobotPosition(140,0,0);
    //~ go_to_straight_line(targetPosition);

    //~ // Build the cakes
    //~ targetPosition = targetPosition + RobotPosition(-30,0,0);
    //~ go_to_straight_line(targetPosition,true);
    //~ setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
    //~ setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
    //~ runActionBlock();
    //~ targetPosition = targetPosition + RobotPosition(100,0,0);
    //~ go_to_straight_line(targetPosition);
    //~ buildCakes();

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

//--------------------------------------------------------------------------------------------------

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
    createdThreads_.clear();
    usleep(50000);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
    }
    funnyAction();
}

//--------------------------------------------------------------------------------------------------

}
