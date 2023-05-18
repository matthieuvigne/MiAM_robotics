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

#define HOMOLOGATION 1

#define RIGHT_ARM_FIRST_SERVO_ID 10
#define LEFT_ARM_FIRST_SERVO_ID 20

#define FUNNY_ACTION_LEFT_SERVO_ID 6
#define FUNNY_ACTION_RIGHT_SERVO_ID 7

#define TEST_CAKE 0
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
    // Empty on purpose*
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
    setTargetPosition(LEFT_ARM, ABS, 0.15, ABS, 10*arm::RAD, ABS, arm::GROUND_HEIGHT + 1e-2);
    setTargetPosition(RIGHT_ARM, ABS, 0.15, ABS, 10*arm::RAD, ABS, arm::GROUND_HEIGHT + 1e-2);
    runActionBlock();
#endif
    isAtBase_ = false;

    std::cout << "End of the setup" << std::endl;
    return true;
}

//--------------------------------------------------------------------------------------------------

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

int Strategy::switch_arm(int arm_idx)
{
  if(robot->isPlayingRightSide())
  {
    switch(arm_idx)
    {
      case LEFT_ARM:
        return RIGHT_ARM;
        break;
      case RIGHT_ARM:
        return LEFT_ARM;
        break;
      default:
        std::cerr << "Unknown pile" << std::endl;
    }
  }
  return arm_idx;
}

//--------------------------------------------------------------------------------------------------

int Strategy::switch_pile(int pile_idx)
{
  if(robot->isPlayingRightSide())
  {
    switch(pile_idx)
    {
      case PILE_IDX::LEFT_SIDE:
        return PILE_IDX::RIGHT_SIDE;
        break;
      case PILE_IDX::LEFT_FRONT:
        return PILE_IDX::RIGHT_FRONT;
        break;
      case PILE_IDX::MIDDLE:
        return PILE_IDX::MIDDLE;
        break;
      case PILE_IDX::RIGHT_FRONT:
        return PILE_IDX::LEFT_FRONT;
        break;
      case PILE_IDX::RIGHT_SIDE:
        return PILE_IDX::LEFT_SIDE;
        break;
      default:
        std::cerr << "Unknown pile" << std::endl;
    }
  }
  return pile_idx;
}

//--------------------------------------------------------------------------------------------------

double Strategy::switch_angle(double angle)
{
  if(robot->isPlayingRightSide())
    angle = - angle;
  return angle;
}

//--------------------------------------------------------------------------------------------------

void Strategy::match_impl()
{
  // Create required variables.
  RobotPosition targetPosition;
  TrajectoryVector traj;
  RobotPosition endPosition;
  std::vector<RobotPosition> positions;
  MotionPlanner* motionPlanner = motionController->motionPlanner_;
  std::vector<Action> actionVector;
  RobotParameters const robotParameters = robot->getParameters();
  RobotPosition tmp_position;
  RobotPosition current_position = motionController->getCurrentPosition();
  
  
  #if HOMOLOGATION
  // Grab first genoise with the arms aside
  clearActionSequence();
  targetPosition = genoese_bottom_left - RobotPosition(robotParameters.CHASSIS_FRONT + 60, 0, 0);
  go_to_straight_line(targetPosition, 1.0);
  // Go between the next two cakes, stop just before them.
  targetPosition = RobotPosition(2000-400,400,0);
  go_to_straight_line(targetPosition, 1.0);
  runActionBlock();
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0., REL, 30*arm::RAD, REL, 0.);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0., REL, 30*arm::RAD, REL, 0.);
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0., REL, -30*arm::RAD, REL, 0.);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0., REL, -30*arm::RAD, REL, 0.);
  runActionBlock();
  robot->wait(7.0);
  go_forward(-1000);
  targetPosition = RobotPosition{750,750,0};
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0., ABS, 0, REL, 0.);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0., ABS, 0, REL, 0.);
  runActionBlock();
  //~ goBackToBase();
  targetPosition = RobotPosition{750,robotParameters.CHASSIS_FRONT + 150,0};
  go_to_straight_line(targetPosition, 1.0);
  funnyAction();
  #else
  // Grab first genoise with the arms aside
  clearActionSequence();
  targetPosition = genoese_bottom_left - RobotPosition(robotParameters.CHASSIS_FRONT + 60, 0, 0);
  go_to_straight_line(targetPosition, 1.5);
  // Go between the next two cakes, stop just before them.
  targetPosition = cream_ganache_bottom_right + RobotPosition(-600,0,0);
  go_to_straight_line(targetPosition, 1.5);
  targetPosition = cream_ganache_bottom_right + RobotPosition(-300,0,0);
  go_to_straight_line(targetPosition, 1.5);
  turn_around_point(switch_angle(5*arm::RAD));
  motionController->resetPosition(targetPosition,true,true,true);
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0.00, ABS, 70*arm::RAD, REL, 0);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0.00, ABS, 70*arm::RAD, REL, 0);
  runActionBlock();
  targetPosition.x = 2000 - cake_radius - robotParameters.CHASSIS_FRONT;
  go_to_straight_line(targetPosition);
  targetPosition.x = targetPosition.x - 100;
  go_to_straight_line(targetPosition, 1.5, true);
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
  runActionBlock();
  targetPosition.x += 40;
  go_to_straight_line(targetPosition, 1.5);
  // Build the cakes
  buildCakes();
  // Push the cakes to safe space
  clearActionSequence();
  
  // [MODIFIED] go_forward(-150);
  targetPosition.x += -150*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.x += -150*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(-150);
  
  turn_around_point(switch_angle(55*arm::RAD));
  
  // [MODIFIED] go_forward(160);
  targetPosition.x += 160*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.y += 160*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(160);
  
  setTargetPosition(switch_arm(LEFT_ARM), ABS, 0.15, ABS, 40*arm::RAD, ABS, arm::GROUND_HEIGHT + 1e-2);
  setTargetPosition(switch_arm(RIGHT_ARM), ABS, 0.15, ABS, 40*arm::RAD, ABS, arm::GROUND_HEIGHT + 1e-2);
  runActionBlock();
  setTargetPosition(switch_arm(LEFT_ARM), REL, 1e-2, REL, -30*arm::RAD, REL, 0.00);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0.00, REL, -30*arm::RAD, REL, 0.00);
  runActionBlock();
  turn_around_point(switch_angle(-90*arm::RAD),0.4);
  turn_around_point(switch_angle(-30*arm::RAD),0.4);
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0.00, REL, 30*arm::RAD, REL, 0.00);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0.00, REL, 30*arm::RAD, REL, 0.00);
  runActionBlock();
  
  // [MODIFIED] go_forward(180);
  targetPosition.x += 180*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.y += 180*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(180);
  
  turn_around_point(switch_angle(-20*arm::RAD),0.4);
  
  // [MODIFIED] go_forward(380);
  targetPosition.x += 380*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.y += 380*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(380);
  
  setTargetPosition(switch_arm(LEFT_ARM), ABS, 0.12, ABS, 90*arm::RAD, REL, 0.00);
  setTargetPosition(switch_arm(RIGHT_ARM), ABS, 0.12, ABS, 90*arm::RAD, REL, 0.00);
  runActionBlock();
  
  // [MODIFIED] go_forward(80);
  targetPosition.x += 80*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.y += 80*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(380);
  
  // Go back to the final zone
  
  // [MODIFIED] go_forward(-500);
  targetPosition.x += -500*std::cos(targetPosition.theta); // [MODIFIED]
  targetPosition.y += -500*std::sin(targetPosition.theta); // [MODIFIED]
  go_to_straight_line(targetPosition); // [MODIFIED] go_forward(380);
  
  targetPosition = RobotPosition{750,750,0};
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0., ABS, 0, REL, 0.);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0., ABS, 0, REL, 0.);
  runActionBlock();
  go_to_straight_line(targetPosition, 1.5, false);
  targetPosition.y -= 500;
  go_to_straight_line(targetPosition, 1.5, false);
  //~ goBackToBase();
  #endif
  
  //~ goBackToBase();

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



void Strategy::goBackToBase()
{
    if (isAtBase_)
    {
        textlog << "[Strategy] already at base" << std::endl;
        return;
    }

    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    RobotPosition position = motionController->getCurrentPosition();
    position.x = 680;
    position.y = 500;
    position.theta = -M_PI_2;

    if ((motionController->getCurrentPosition() - position).norm() > 100)
    {
        traj = robot->getMotionController()->computeMPCTrajectory(
            position,
            robot->getMotionController()->getDetectedObstacles(),
            false,   // is forward
            true,   // is an avoidance traj
            true); // ensure end angle
        robot->getMotionController()->setTrajectoryToFollow(traj);
        robot->getMotionController()->waitForTrajectoryFinished();
    }
    else
    {
        go_to_straight_line(position);
    }

    position.x = 680;
    position.y = 100;
    position.theta = -M_PI_2;

    TrajectoryConfig trajconf = robot->getParameters().getTrajConf();
    trajconf.maxWheelVelocity = 200;

    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
        trajconf,
        motionController->getCurrentPosition(), // start
        position, // end
        0.0, // no velocity at end point
        false // backward
    );

    for (auto subtraj : traj)
    {
        subtraj->setAvoidanceEnabled(true);
    }

    robot->getMotionController()->setTrajectoryToFollow(traj);
    robot->getMotionController()->waitForTrajectoryFinished();

    isAtBase_ = true;

}

}
