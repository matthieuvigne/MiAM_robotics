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
    std::cout << "Before mutex" << std::endl;
    pileLock.lock();
    pileHeight[pileIndex] += delta;
    if (pileHeight[pileIndex] < 0)
        std::cout << ">>>>> WARNING pileHeight[pileIndex] is <0 <<<<<" << std::endl;
    pileLock.unlock();
    std::cout << "After mutex" << std::endl;
}

//--------------------------------------------------------------------------------------------------

int Strategy::getPileHeight(int pileIndex)
{
    pileLock.lock();
    int height = pileHeight[pileIndex];
    std::cout << "Pile state is " 
      << pileHeight[PILE_IDX::LEFT_SIDE] << " " 
      << pileHeight[PILE_IDX::LEFT_FRONT] << " " 
      << pileHeight[PILE_IDX::MIDDLE] << " " 
      << pileHeight[PILE_IDX::RIGHT_FRONT] << " " 
      << pileHeight[PILE_IDX::RIGHT_SIDE] << std::endl;
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

void Strategy::buildCakes()
{
#ifdef SIMULATION
    robot->wait(10.0);
    return;
#endif

    std::cout << "Building cakes" << std::endl;
    
    // Right arm has inverted angles!
    ArmPosition middlePile(arm::CAKES_FRONT_DISTANCE + 1e-2, arm::MIDDLE_PILE_ANGLE, arm::GROUND_HEIGHT + 60e-3);
    ArmPosition frontPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_PILE_ANGLE, arm::GROUND_HEIGHT + 60e-3);
    ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_PILE_ANGLE, arm::GROUND_HEIGHT + 40e-3);

    // Reset piles heights
    pileLock.lock();
    pileHeight[PILE_IDX::LEFT_SIDE] = 0;
    pileHeight[PILE_IDX::LEFT_FRONT] = 3;
    pileHeight[PILE_IDX::MIDDLE] = 3;
    pileHeight[PILE_IDX::RIGHT_FRONT] = 3;
    pileHeight[PILE_IDX::RIGHT_SIDE] = 0;
    pileLock.unlock();
    
    // Get the current position of both arms
    last_left_position = getArmPosition(LEFT_ARM);
    last_right_position = getArmPosition(RIGHT_ARM);
    std::cout << "Initial left position: " << last_left_position << std::endl;
    std::cout << "Initial right position: " << last_right_position << std::endl;
    initPosition(LEFT_ARM, last_left_position);
    initPosition(RIGHT_ARM, last_right_position);
        
    // First synchronize block
    // -----------------------
    // Left arm turns its pump on, and goes down to take a genoese and goes up
    std::cout << ">>>>>>>>>>>>>>>>> First block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_ + 10e-3, ABS, middlePile.theta_, ABS, PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 3e-3);
    setTargetPosition(LEFT_ARM, REL, -10e-3, REL, 0, REL, 0);
    oscillate(LEFT_ARM, 3*arm::RAD);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 2e-3);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 5e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, REL, 10e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, ABS, PILE_CLEAR_HEIGHT + 10e-3);
    setTargetPosition(LEFT_ARM, REL, 0, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, REL, 0, REL, 0);
    // Left arm goes to side pile and delivers its genoese
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_SIDE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    runActionBlock();
        
    // Second synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Second block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes cream from its front pile and delivers it to its side pile
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 3e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 1e-3);
    setTargetPosition(LEFT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT));
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 2e-3);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 5e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, REL, 10e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, ABS, sidePile.theta_-8*arm::RAD, REL, 0);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, REL, 0, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_SIDE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Right arm takes ganache from its front pile and delivers it to the middle pile
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT));
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(RIGHT_ARM, REL, -4e-3, REL, 0, REL, 2e-3);
    setTargetPosition(RIGHT_ARM, REL, -4e-3, REL, 0, REL, 5e-3);
    setTargetPosition(RIGHT_ARM, REL, -3e-3, REL, 0, REL, 10e-3);
    setTargetPosition(RIGHT_ARM, REL, -3e-3, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 5e-3, REL, 0, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Run the block
    runActionBlock();
    
    // Third synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Third block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes the ganache from the middle pile
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_ + 10e-3, ABS, middlePile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 3e-3);
    setTargetPosition(LEFT_ARM, REL, -10e-3, REL, 0, REL, 0);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 2e-3);
    setTargetPosition(LEFT_ARM, REL, -4e-3, REL, 0, REL, 5e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, REL, 10e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, REL, PILE_CLEAR_HEIGHT); // [PROBLEM]
    setTargetPosition(LEFT_ARM, REL, 0, ABS, sidePile.theta_-8*arm::RAD, REL, 0);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, REL, 0, REL, 0);
    // Left arm delivers it to its side pile
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_SIDE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Right arm goes over the ganache
    setTargetPosition(RIGHT_ARM, REL, 0, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    // Synchronize
    runActionBlock();
    std::cout << "END OF BLOCK" << std::endl;
    while(true);;
    
    // Fourth synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fourth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Right arm takes genoese from middle pile and rises it up
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 10e-3, ABS, middlePile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE));
    changePileHeight(PILE_IDX::MIDDLE, -1);
    oscillate(RIGHT_ARM, 3*arm::RAD);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 7e-3);
    // Left arm takes cream from front pile and rises it up
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 3e-3);
    wait(LEFT_ARM, 3.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 1e-3);
    setTargetPosition(LEFT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT));
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 7e-3);
    // Synchronize
    runActionBlock();
    
    // Fifth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fifth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm delivers the cream to the middle pile
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Right arm delivers the genoese to the side pile
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Synchronize
    runActionBlock();

    // Sixth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sixth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Right arm takes cream from middle pile and rises it up
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 10e-3, ABS, middlePile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE));
    changePileHeight(PILE_IDX::MIDDLE, -1);
    oscillate(RIGHT_ARM, 3*arm::RAD);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    // Left arm takes cream from front pile and rises it up
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 3e-3);
    wait(LEFT_ARM, 3.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 1e-3);
    setTargetPosition(LEFT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT));
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    // Synchronize
    runActionBlock();

    // Seventh synchronize block
    // -------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sventh block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Right arm delivers cream to its side pile
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Left arm delivers cream to the middle pile
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Synchronize
    runActionBlock();
    
    // Eighth synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Eighth block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes from the middle pile to its side pile
    setTargetPosition(LEFT_ARM, REL, 0, REL, sidePile.theta_, REL, 0);
    // Right arms takes ganache from front pile and delivers it to middle pile
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT));
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 5e-3, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Right arms takes ganache from front pile and delivers it to side pile
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_ + 10e-3, ABS, frontPile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, -10e-3, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT));
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 2e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, REL, 10e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_ + 5e-3, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Synchronize
    runActionBlock();
    std::cout << "END OF BLOCK" << std::endl;
    while(true);;
}

//--------------------------------------------------------------------------------------------------
