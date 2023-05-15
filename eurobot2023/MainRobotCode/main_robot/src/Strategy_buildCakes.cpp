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

double Strategy::getPileZ(int pileIndex)
{
  return arm::GROUND_HEIGHT + getPileHeight(pileIndex) * arm::LAYER_HEIGHT;
}

void Strategy::addPositionToQueue_Left(ArmPosition target)
{
    std::vector<ArmPosition::Ptr > seq = computeSequenceToPosition(LEFT_ARM, target);
    for (auto& ap : seq)
    {
        left_arm_positions.push(ap);
    }
}

void Strategy::addPositionToQueue_Right(ArmPosition target)
{
    std::vector<ArmPosition::Ptr > seq = computeSequenceToPosition(RIGHT_ARM, target);
    for (auto& ap : seq)
    {
        right_arm_positions.push(ap);
    }
}

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

void Strategy::initPosition(int arm_idx, ArmPosition const& position)
{
  initPosition(arm_idx, position.r_, position.theta_, position.z_);
}

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
      std::cout << "Unknown arm type" << std::endl;
  }
}

void Strategy::setRelTargetPosition(
  int arm_idx, double dr, double dtheta_rad, double dz)
{
  return setTargetPosition(arm_idx, REL, dr, REL, dtheta_rad, REL, dz);
}

void Strategy::setAbsTargetPosition(
  int arm_idx, double new_r, double new_theta_rad, double new_z)
{ 
  return setTargetPosition(arm_idx, ABS, new_r, ABS, new_theta_rad, ABS, new_z);
}

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

void Strategy::runActionBlock()
{
  ArmSync::Ptr left_sync(new ArmSync());
  left_arm_positions.push(left_sync);
  ArmSync::Ptr right_sync(new ArmSync());
  right_arm_positions.push(right_sync);
  waitForArmMotionSequenced();
}

void Strategy::addSyncToQueue()
{
    ArmSync::Ptr target(new ArmSync());
    left_arm_positions.push(target);
    ArmSync::Ptr target2(new ArmSync());
    right_arm_positions.push(target2);
}

void Strategy::addPumpToLeftQueue(bool activated)
{
    ArmPump::Ptr target(new ArmPump(activated));
    left_arm_positions.push(target);
}

void Strategy::addPumpToRightQueue(bool activated)
{
    ArmPump::Ptr target(new ArmPump(activated));
    right_arm_positions.push(target);
}

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

    // Initialize both arms
    // --------------------
    // Chicken !!
    std::cout << ">>>>>>>>>>>>>>>>> Initialization  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, ABS, PILE_CLEAR_HEIGHT);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, ABS, PILE_CLEAR_HEIGHT);
    wait(LEFT_ARM, 1.5);
    wait(RIGHT_ARM, 1.5);
    runActionBlock();
        
    // First synchronize block
    // -----------------------
    // Left arm turns its pump on, and goes down to take a genoese and goes up
    std::cout << ">>>>>>>>>>>>>>>>> First block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, ABS, PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 3e-3);
    setTargetPosition(LEFT_ARM, REL, -3e-3, REL, 0, REL, 0);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    runActionBlock();
    
    // Second synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Second block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes to side pile and delivers its genoese
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_SIDE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Right arm goes to center, turns its pump on and takes its genoese
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 3e-3, ABS, middlePile.theta_, ABS, arm::PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 3e-3, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, -3e-3, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 2e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0.03, REL, 0);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    wait(RIGHT_ARM, 1.0);
    // Right arm goes to side pile and delivers its genoese
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, REL, 5e-3, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 2e-2);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 1.0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, arm::PILE_CLEAR_HEIGHT);
    // Run the block
    runActionBlock();
    
    // Third synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Third block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes to front pile and takes a cream, and delivers it on the middle pile
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 8e-3, ABS, frontPile.theta_, ABS, PILE_CLEAR_HEIGHT);
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) - 3e-3);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    setTargetPosition(LEFT_ARM, REL, 0, ABS, middlePile.theta_ - 0.08, REL, 0);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 3e-2);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 2e-2);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 1.0);
    // Left arm returns to the left front pile
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_, ABS, frontPile.theta_, REL, 0);
    // Synchronize
    runActionBlock();
    
    // Fourth synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fourth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes a cream from the front pile
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 3e-3, ABS, frontPile.theta_, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 3e-3);
    setTargetPosition(LEFT_ARM, ABS, frontPile.r_ + 3e-3, ABS, frontPile.theta_, ABS, getPileZ(PILE_IDX::LEFT_FRONT) - 3e-3);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    // Left arm delivers the cream to the side pile
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, ABS, sidePile.theta_, REL, 0);
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 0.5);
    // Left arm goes over its front pile
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, ABS, frontPile.theta_, REL, 0);
    // Right arm takes the cream from the middle pile
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_ + 3e-3, ABS, middlePile.theta_, ABS, PILE_CLEAR_HEIGHT);
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 3e-3);
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    // Right arm delivers it to the side pile
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 5e-3);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 0.5);
    // Right arm goes over its front pile
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_, ABS, frontPile.theta_, REL, 0);
    // Synchronize
    runActionBlock();
    
    // Fifth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fifth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm moves cream from its front pile to the middle pile
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) + 3e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_FRONT) - 3e-3);
    wait(LEFT_ARM, 1.0);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    // Left arm goes to the middle pile and delivers the cream
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE));
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 0.5);
    // Left arm goes over the front pile
    setTargetPosition(LEFT_ARM, REL, 0, ABS, frontPile.theta_, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Right arm picks the ganache from its front pile
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT));
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Synchronize
    runActionBlock();

    // Sixth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sixth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Right arm delivers ganache to the middle pile
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 0.5);
    // Right arm returns over its front pile
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    // Synchronize
    runActionBlock();

    // Seventh synchronize block
    // -------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sventh block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes the ganache from the middle pile
    pump(LEFT_ARM, true);
    setTargetPosition(LEFT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) - 6e-3);
    wait(LEFT_ARM, 0.5);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    // Left arm delivers the ganache to the side pile
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT);
    setTargetPosition(LEFT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::LEFT_SIDE));
    pump(LEFT_ARM, false);
    wait(LEFT_ARM, 0.5);
    setTargetPosition(LEFT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Right arm moves ganache from front pile to side pile
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_, ABS, frontPile.theta_, REL, 0);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 1e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) - 5e-3);
    wait(RIGHT_ARM, 2.0);
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Right arm delivers the ganache to the middle pile
    setTargetPosition(RIGHT_ARM, ABS, middlePile.r_, ABS, middlePile.theta_, REL, 0);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::MIDDLE) + 5e-3);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 0.5);
    // Right arm moves over front pile
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, frontPile.r_, ABS, frontPile.theta_, REL, 0);
    // Right arm moves ganache from front pile to side pile
    pump(RIGHT_ARM, true);
    setTargetPosition(RIGHT_ARM, REL, 5e-3, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) + 3e-3);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_FRONT) - 5e-3);
    wait(RIGHT_ARM, 1.0);
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    setTargetPosition(RIGHT_ARM, ABS, sidePile.r_, ABS, sidePile.theta_, REL, 0);
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, getPileZ(PILE_IDX::RIGHT_SIDE) + 5e-3);
    pump(RIGHT_ARM, false);
    wait(RIGHT_ARM, 0.5);
    setTargetPosition(RIGHT_ARM, REL, 0, REL, 0, ABS, PILE_CLEAR_HEIGHT + 5e-3);
    // Synchronize
    runActionBlock();
    std::cout << "FINISHED" << std::endl;
    while(true);;
    
    // std::vector<std::shared_ptr<ArmPosition > > seq = computeSequenceToPosition(LEFT_ARM, sidePile);
    // for (auto& ap : seq)
    // {
    //     left_arm_positions.push(ap);
    // }

    // waitForArmMotionSequenced();



    // std::cout << "middlePile: " << middlePile.r_ << " " << middlePile.theta_ << " " << middlePile.z_ << std::endl;


    // std::array<double, 4 > destArray;
    // common::arm_inverse_kinematics(middlePile.r_, middlePile.theta_, middlePile.z_, -M_PI_2, &destArray);
    // ArmPosition position_tmp = servoAnglesToArmPosition(destArray[0], destArray[1], destArray[2], destArray[3]);
    // std::cout << "position_tmp: " << position_tmp.r_ << " " << position_tmp.theta_ << " " << position_tmp.z_ << std::endl;

    // // left arm
    // {
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(middlePile));
    //     left_arm_positions.push (targetPosition);
    // }

    // waitForArmMotionSequenced();

    // ArmPosition leftArm = getArmPosition(LEFT_ARM);
    // std::cout << "leftArm: " << leftArm.r_ << " " << leftArm.theta_ << " " << leftArm.z_ << std::endl;

    // ArmPosition rightArm = getArmPosition(RIGHT_ARM);
    // std::cout << "rightArm: " << rightArm.r_ << " " << rightArm.theta_ << " " << rightArm.z_ << std::endl;

    // while(true) ;;

    // Grab first item on right pile with left arm

    // // left arm
    // {
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(frontPile));
    //     targetPosition->z_ += LAYER_MOVEMENT_CLEARANCE;
    //     left_arm_positions.push (targetPosition);
    // }

    // {
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(frontPile));
    //     left_arm_positions.push (targetPosition);
    // }

    // waitForArmMotionSequenced();

    // std::cout << "Sequence finished" << std::endl;

    //~ return;





    //~ // go above right pile
    //~ ArmPosition targetPosition = frontPile;
    //~ targetPosition.z_ += LAYER_MOVEMENT_CLEARANCE;
    //~ setArmPosition(LEFT_ARM, targetPosition);
    //~ waitForArmMotion();

    //~ // grab
    //~ targetPosition.z_ -= LAYER_MOVEMENT_CLEARANCE;
    //~ setArmPosition(LEFT_ARM, targetPosition);
    //~ robot->wait(1.5);
    //~ targetPosition.z_ = arm::PILE_CLEAR_HEIGHT;
    //~ setArmPosition(LEFT_ARM, targetPosition);
    //~ waitForArmMotion();




    //~ targetPosition.theta_ = sidePile.theta_;
    //~ setArmPosition(LEFT_ARM, targetPosition);
    //~ waitForArmMotion();
    //~ targetPosition.z_ = arm::GROUND_HEIGHT + 0.035;
    //~ setArmPosition(LEFT_ARM, targetPosition);
    //~ waitForArmMotion();
    //~ RPi_writeGPIO(PUMP_LEFT, false);
    //~ RPi_writeGPIO(VALVE_LEFT, true);
    //~ robot->wait(0.1);
    //~ RPi_writeGPIO(VALVE_LEFT, false);



    // // Grab first item on right pile with left arm
    // ArmPosition targetPosition = frontPile;
    // targetPosition.z_ += LAYER_MOVEMENT_CLEARANCE;
    // setArmPosition(LEFT_ARM, targetPosition);
    // waitForArmMotion();
    // RPi_writeGPIO(PUMP_LEFT, true);
    // RPi_writeGPIO(VALVE_LEFT, false);
    // targetPosition.z_ -= 0.025;
    // setArmPosition(LEFT_ARM, targetPosition);
    // robot->wait(1.5);
    // targetPosition.z_ = arm::PILE_CLEAR_HEIGHT;
    // setArmPosition(LEFT_ARM, targetPosition);
    // waitForArmMotion();
    // targetPosition.theta_ = sidePile.theta_;
    // setArmPosition(LEFT_ARM, targetPosition);
    // waitForArmMotion();
    // targetPosition.z_ = arm::GROUND_HEIGHT + 0.035;
    // setArmPosition(LEFT_ARM, targetPosition);
    // waitForArmMotion();
    // RPi_writeGPIO(PUMP_LEFT, false);
    // RPi_writeGPIO(VALVE_LEFT, true);
    // robot->wait(0.1);
    // RPi_writeGPIO(VALVE_LEFT, false);

}
