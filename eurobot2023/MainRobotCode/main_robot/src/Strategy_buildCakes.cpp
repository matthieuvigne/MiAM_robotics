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

using namespace main_robot;
using namespace main_robot::arm;

//--------------------------------------------------------------------------------------------------

void Strategy::changePileHeight(int pileIndex, int delta)
{
    pileLock.lock();
    pileHeight[pileIndex] += delta;
    if (pileHeight[pileIndex] < 0)
        std::cout << ">>>>> WARNING pileHeight[pileIndex] is <0 <<<<<" << std::endl;
    pileLock.unlock();
}

//--------------------------------------------------------------------------------------------------

int Strategy::getPileHeight(int pileIndex)
{
    pileLock.lock();
    int height = pileHeight[pileIndex];
    pileLock.unlock();
    return height;
}

//--------------------------------------------------------------------------------------------------

double Strategy::getPileZ(int pileIndex)
{
  return arm::GROUND_HEIGHT + getPileHeight(pileIndex) * arm::LAYER_HEIGHT;
}

//--------------------------------------------------------------------------------------------------

void Strategy::addPositionToQueue_Left(ArmPosition target)
{
    std::vector<ArmPosition::Ptr > seq = computeSequenceToPosition(LEFT_ARM, target);
    for (auto& ap : seq)
    {
        left_arm_positions.push(ap);
    }
}

//--------------------------------------------------------------------------------------------------

void Strategy::addPositionToQueue_Right(ArmPosition target)
{
    std::vector<ArmPosition::Ptr > seq = computeSequenceToPosition(RIGHT_ARM, target);
    for (auto& ap : seq)
    {
        right_arm_positions.push(ap);
    }
}

//--------------------------------------------------------------------------------------------------

void Strategy::initPosition(int arm_idx, double r, double theta_rad, double z)
{
  switch(arm_idx)
  {
    case LEFT_ARM:
      left_arm_position_.r_ = r;
      left_arm_position_.theta_ = theta_rad;
      left_arm_position_.z_ = z;
      break;
    case RIGHT_ARM:
      right_arm_position_.r_ = r;
      right_arm_position_.theta_ = theta_rad;
      right_arm_position_.z_ = z;
      break;
    default:
      std::cout << "Unknown arm type" << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

void Strategy::initPosition(int arm_idx, ArmPosition const& position)
{
  initPosition(arm_idx, position.r_, position.theta_, position.z_);
}

//--------------------------------------------------------------------------------------------------

void Strategy::setTargetPosition(int arm_idx, 
  int absrel_r, double r, 
  int absrel_theta, double theta_rad, 
  int absrel_z, double z)
{
  switch(arm_idx)
  {
    case LEFT_ARM:
      left_arm_position_.r_ = absrel_r*left_arm_position_.r_ + r;
      left_arm_position_.theta_ = absrel_theta*left_arm_position_.theta_ + theta_rad;
      left_arm_position_.z_  = absrel_z*left_arm_position_.z_ + z;
      addPositionToQueue_Left(left_arm_position_);
      break;
    case RIGHT_ARM:
      right_arm_position_.r_ = absrel_r*right_arm_position_.r_ + r;
      right_arm_position_.theta_ = absrel_theta*right_arm_position_.theta_ + theta_rad;
      right_arm_position_.z_  = absrel_z*right_arm_position_.z_ + z;
      addPositionToQueue_Right(right_arm_position_);
      break;
    default:
      std::cerr << "Unknown arm type" << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

void Strategy::setTargetPositionTicks(int arm_idx, 
  int16_t tick0, int16_t tick1, int16_t tick2, int16_t tick3)
{
  switch(arm_idx)
  {
    case LEFT_ARM:
      left_arm_position_ = arm::servoAnglesToArmPosition(
        STS::servoToRadValue(tick0), STS::servoToRadValue(tick1),
        STS::servoToRadValue(tick2), STS::servoToRadValue(tick3));
      addPositionToQueue_Left(left_arm_position_);
      break;
    case RIGHT_ARM:
      right_arm_position_ = arm::servoAnglesToArmPosition(
        STS::servoToRadValue(tick0), STS::servoToRadValue(tick1),
        STS::servoToRadValue(tick2), STS::servoToRadValue(tick3));
      addPositionToQueue_Right(right_arm_position_);
      break;
    default:
      std::cerr << "Unknown arm type" << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

void Strategy::wait(int arm_idx, double duration_sec)
{
  ArmAction::Ptr action(new ArmWait(duration_sec));
  switch(arm_idx)
  {
    case LEFT_ARM:
      left_arm_positions.push(action);
      break;
    case RIGHT_ARM:
      right_arm_positions.push(action);
      break;
    default:
      std::cout << "Unknown arm type" << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

void Strategy::pump(int arm_idx, bool activate)
{
  ArmAction::Ptr action(new ArmPump(activate));
  switch(arm_idx)
  {
    case LEFT_ARM:
      left_arm_positions.push(action);
      break;
    case RIGHT_ARM:
      right_arm_positions.push(action);
      break;
    default:
      std::cout << "Unknown arm type" << std::endl;
  }
}

//--------------------------------------------------------------------------------------------------

void Strategy::runActionBlock()
{
  ArmSync::Ptr left_sync(new ArmSync());
  left_arm_positions.push(left_sync);
  ArmSync::Ptr right_sync(new ArmSync());
  right_arm_positions.push(right_sync);
  waitForArmMotionSequenced();
}

//--------------------------------------------------------------------------------------------------

void Strategy::clearActionSequence()
{
  while(!left_arm_positions.empty())
    left_arm_positions.pop();
  while(!right_arm_positions.empty())
    right_arm_positions.pop();
}

//--------------------------------------------------------------------------------------------------

void Strategy::addSyncToQueue()
{
    ArmSync::Ptr target(new ArmSync());
    left_arm_positions.push(target);
    ArmSync::Ptr target2(new ArmSync());
    right_arm_positions.push(target2);
}

//--------------------------------------------------------------------------------------------------

void Strategy::addPumpToLeftQueue(bool activated)
{
    ArmPump::Ptr target(new ArmPump(activated));
    left_arm_positions.push(target);
}

//--------------------------------------------------------------------------------------------------

void Strategy::addPumpToRightQueue(bool activated)
{
    ArmPump::Ptr target(new ArmPump(activated));
    right_arm_positions.push(target);
}

//--------------------------------------------------------------------------------------------------

void Strategy::oscillate(int arm_idx, double amplitude_rad)
{
  setTargetPosition(arm_idx, REL, 0, REL, amplitude_rad, REL, 0);
  setTargetPosition(arm_idx, REL, 0, REL, -2*amplitude_rad, REL, 0);
  setTargetPosition(arm_idx, REL, 0, REL, amplitude_rad, REL, 0);
}

//--------------------------------------------------------------------------------------------------

void Strategy::resetPileHeights()
{
  pileLock.lock();
  pileHeight[PILE_IDX::LEFT_SIDE] = 0;
  pileHeight[PILE_IDX::LEFT_FRONT] = 3;
  pileHeight[PILE_IDX::MIDDLE] = 3;
  pileHeight[PILE_IDX::RIGHT_FRONT] = 3;
  pileHeight[PILE_IDX::RIGHT_SIDE] = 0;
  pileLock.unlock();
}

//--------------------------------------------------------------------------------------------------

ArmPosition Strategy::getPileFromIndex(int pile_idx)
{
  ArmPosition pile;
  switch(pile_idx)
  {
    case PILE_IDX::LEFT_SIDE:
    case PILE_IDX::RIGHT_SIDE:
      pile = sidePile;
      break;
    case PILE_IDX::LEFT_FRONT:
    case PILE_IDX::RIGHT_FRONT:
      pile = frontPile;
      break;
    case PILE_IDX::MIDDLE:
      pile = middlePile;
      break;
    default:
      throw std::runtime_error("Unknown pile");
  }
  return pile;
}

//--------------------------------------------------------------------------------------------------

void Strategy::takeCherry()
{  
  ArmPosition position{0.100,-1.05,-0.125};
  setTargetPosition(LEFT_ARM, ABS, 0.130, ABS, position.theta_, ABS, position.z_);
  setTargetPosition(LEFT_ARM, ABS, position.r_, ABS, position.theta_, ABS, position.z_);
  setTargetPosition(LEFT_ARM, ABS, position.r_, ABS, -1.115, ABS, position.z_);
  setTargetPosition(LEFT_ARM, ABS, position.r_, ABS, -1.115, REL, -15e-3);
  pump(LEFT_ARM, true);
  wait(LEFT_ARM, 2.0);
  setTargetPosition(LEFT_ARM, REL, 1e-2, REL, 5*arm::RAD, REL, 2e-2);
  runActionBlock();
}

//--------------------------------------------------------------------------------------------------

void Strategy::grabCakeFromPile(int arm_idx, int pile_idx, bool oscillate, double wait_sec)
{
  // Update arm and pile according to current playing side
  arm_idx = switch_arm(arm_idx);
  pile_idx = switch_pile(pile_idx);
  
  // Get specific corrections
  double delta_r = (getPileHeight(pile_idx)==1) ? 10e-3 : 0.; // before 5e-3
  delta_r = (getPileHeight(pile_idx)==2) ? 5e-3 : delta_r; // [ADDED]
  //~ delta_r = (pile_idx==PILE_IDX::MIDDLE) ? XX : delta_r;
  double delta_theta1 = (getPileHeight(pile_idx)==1) ? 20*arm::RAD : 0.;
  double delta_theta2 = (getPileHeight(pile_idx)==1) ? 10*arm::RAD : 0.;
  double delta_z = (getPileHeight(pile_idx)==1) ? 2e-3 : 0.; // [ADDED]
  
  // Wait if requested
  if(wait_sec > 0.) wait(arm_idx, wait_sec);
  
  // Perform requested action
  ArmPosition const pile = getPileFromIndex(pile_idx);
  setTargetPosition(arm_idx, ABS, pile.r_ + 10e-3 + delta_r, ABS, pile.theta_ + delta_theta2, ABS, PILE_CLEAR_HEIGHT);
  pump(arm_idx, true);
  setTargetPosition(arm_idx, REL, 0, REL, 0, ABS, getPileZ(pile_idx) + 5e-3);
  setTargetPosition(arm_idx, REL, 0, REL, 0, ABS, getPileZ(pile_idx) - 3e-3 + delta_z);
  setTargetPosition(arm_idx, REL, -10e-3, REL, delta_theta1, REL, 0);
  if(oscillate) this->oscillate(arm_idx, 3*arm::RAD);
  wait(arm_idx, 0.250);
  changePileHeight(pile_idx, -1);
  setTargetPosition(arm_idx, REL, -4e-3, REL, 0, REL, 2e-3);
  setTargetPosition(arm_idx, REL, -4e-3, REL, 0, REL, 5e-3);
  setTargetPosition(arm_idx, REL, -3e-3, REL, 0, REL, 10e-3);
  setTargetPosition(arm_idx, REL, -3e-3, REL, 0, ABS, PILE_CLEAR_HEIGHT + 10e-3);
}

//--------------------------------------------------------------------------------------------------

void Strategy::dumbCakeToPile(int arm_idx, int pile_idx)
{
  // Update arm and pile according to current playing side
  arm_idx = switch_arm(arm_idx);
  pile_idx = switch_pile(pile_idx);
  
  // Perform requested action
  double delta_r = (pile_idx == PILE_IDX::MIDDLE) ? 15e-3 : 0.;
  ArmPosition const pile = getPileFromIndex(pile_idx);
  setTargetPosition(arm_idx, REL, 0, ABS, pile.theta_, REL, 0);
  setTargetPosition(arm_idx, ABS, pile.r_, REL, 0, REL, 0);
  setTargetPosition(arm_idx, REL, delta_r, REL, 0, ABS, getPileZ(pile_idx) + LAYER_HEIGHT + 2e-2);
  pump(arm_idx, false);
  wait(arm_idx, 0.150);
  changePileHeight(pile_idx, 1);
  setTargetPosition(arm_idx, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
}

//--------------------------------------------------------------------------------------------------

void Strategy::adjustRobotPosition()
{
  RobotPosition pos_before = motionController->getCurrentPosition();
  go_to_straight_line(pos_before + RobotPosition(40,0,0));
  //~ go_forward(40);
  RobotPosition pos_after = motionController->getCurrentPosition();
  double delta_x = pos_after.x - pos_before.x;
  robot->wait(0.100);
  go_to_straight_line(pos_after + RobotPosition(-20,0,0), 1.0, true);
  //~ go_forward(-delta_x);
  //~ go_forward(-20);
}

//--------------------------------------------------------------------------------------------------

void Strategy::buildCakes()
{
  #ifdef SIMULATION
      robot->wait(10.0);
      return;
  #endif

  std::cout << "Building cakes" << std::endl;
  std::cout << "LEFT_ARM: " << LEFT_ARM << std::endl;
  std::cout << "RIGHT_ARM: " << RIGHT_ARM << std::endl;
  
  // Right arm has inverted angles!
  ArmPosition middlePile(
    arm::CAKES_FRONT_DISTANCE + 10e-3, 
    arm::MIDDLE_PILE_ANGLE,
    arm::GROUND_HEIGHT + 60e-3);
  ArmPosition frontPile(
    arm::CAKES_FRONT_DISTANCE,
    arm::FRONT_PILE_ANGLE,
    arm::GROUND_HEIGHT + 60e-3);
  ArmPosition sidePile(
    arm::CAKES_SIDE_DISTANCE,
    arm::SIDE_PILE_ANGLE,
    arm::GROUND_HEIGHT + 40e-3);

  // Reset piles heights
  resetPileHeights();
  
  // Get the current position of both arms
  last_left_position = getArmPosition(LEFT_ARM);
  last_right_position = getArmPosition(RIGHT_ARM);
  std::cout << "Initial left position: " << last_left_position << std::endl;
  std::cout << "Initial right position: " << last_right_position << std::endl;
  initPosition(LEFT_ARM, last_left_position);
  initPosition(RIGHT_ARM, last_right_position);
      
  // Block 1
  // Left arm grabs genoese from middle pile and dumbs it to its side pile
  grabCakeFromPile(LEFT_ARM, PILE_IDX::MIDDLE, true);
  dumbCakeToPile(LEFT_ARM, PILE_IDX::LEFT_SIDE);
  runActionBlock();
  
  // Block 2
  // Left arm takes cream from its front pile and delivers it to its side pile
  // Right arm takes ganache from its front pile and delivers it to the middle pile
  grabCakeFromPile(LEFT_ARM, PILE_IDX::LEFT_FRONT, false);
  dumbCakeToPile(LEFT_ARM, PILE_IDX::LEFT_SIDE);
  grabCakeFromPile(RIGHT_ARM, PILE_IDX::RIGHT_FRONT, true);
  dumbCakeToPile(RIGHT_ARM, PILE_IDX::MIDDLE);
  runActionBlock();
  adjustRobotPosition();
  
  // Block 3
  // Left arm grabs ganache from middle and dumbs it to side
  // Right arm goes over ganache
  grabCakeFromPile(LEFT_ARM, PILE_IDX::MIDDLE, false);
  dumbCakeToPile(LEFT_ARM, PILE_IDX::LEFT_SIDE);
  setTargetPosition(switch_arm(RIGHT_ARM), REL, 0, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
  runActionBlock();
  adjustRobotPosition(); // [ADDED ven. 11:58 -> REMOVE ?]
  
  // Block 4
  // Left arm takes cream from front pile and rises it up
  // Right arm takes genoese from middle pile and rises it up
  grabCakeFromPile(LEFT_ARM, PILE_IDX::LEFT_FRONT, true);
  grabCakeFromPile(RIGHT_ARM, PILE_IDX::MIDDLE, true, 1.0);
  runActionBlock();
  
  // Block 5
  // Left arm delivers the cream to the middle pile
  // Right arm delivers the genoese to the side pile
  dumbCakeToPile(LEFT_ARM, PILE_IDX::MIDDLE);
  dumbCakeToPile(RIGHT_ARM, PILE_IDX::RIGHT_SIDE);
  runActionBlock();
  adjustRobotPosition();
  
  // Block 6
  // Left arm takes cream from front pile and rises it up
  // Right arm takes cream from middle pile and rises it up
  grabCakeFromPile(LEFT_ARM, PILE_IDX::LEFT_FRONT, false);
  grabCakeFromPile(RIGHT_ARM, PILE_IDX::MIDDLE, false);
  runActionBlock();
  
  // Block 7
  // Left arm delivers cream to the middle pile
  // Right arm delivers cream to its side pile
  dumbCakeToPile(LEFT_ARM, PILE_IDX::MIDDLE);
  dumbCakeToPile(RIGHT_ARM, PILE_IDX::RIGHT_SIDE);
  runActionBlock();
  adjustRobotPosition();
  
  // Block 8 [REVERSE IF CHERRY]
  // Left arm goes from the middle pile to its side pile
  // Right arms takes ganache from front pile and delivers it to middle pile
  setTargetPosition(switch_arm(LEFT_ARM), REL, 0, REL, sidePile.theta_, REL, 0);
  grabCakeFromPile(RIGHT_ARM, PILE_IDX::RIGHT_FRONT, false);
  dumbCakeToPile(RIGHT_ARM, PILE_IDX::MIDDLE);
  runActionBlock();
  adjustRobotPosition();
  
  // Block 9 [REVERSE IF CHERRY]
  // Right arms takes ganache from front pile and delivers it to side pile
  grabCakeFromPile(RIGHT_ARM, PILE_IDX::RIGHT_FRONT, false);
  dumbCakeToPile(RIGHT_ARM, PILE_IDX::RIGHT_SIDE);
  runActionBlock();
}

//--------------------------------------------------------------------------------------------------
