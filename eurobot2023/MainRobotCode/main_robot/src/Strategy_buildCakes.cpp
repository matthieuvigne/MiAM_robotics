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
    std::cout << "Building cakes" << std::endl;
    
    // right arm has inverted angles!
    ArmPosition middlePile(arm::CAKES_FRONT_DISTANCE, arm::MIDDLE_PILE_ANGLE, arm::GROUND_HEIGHT + 60e-3);
    ArmPosition frontPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_PILE_ANGLE, arm::GROUND_HEIGHT + 60e-3);
    ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_PILE_ANGLE, arm::GROUND_HEIGHT + 40e-3);

    // reset piles and arm positions

    pileLock.lock();
    pileHeight[PILE_IDX::LEFT_SIDE] = 0;
    pileHeight[PILE_IDX::LEFT_FRONT] = 3;
    pileHeight[PILE_IDX::MIDDLE] = 3;
    pileHeight[PILE_IDX::RIGHT_FRONT] = 3;
    pileHeight[PILE_IDX::RIGHT_SIDE] = 0;
    pileLock.unlock();
    
    last_left_position = getArmPosition(LEFT_ARM);
    last_right_position = getArmPosition(RIGHT_ARM);

    std::cout << "Initial left position: " << last_left_position << std::endl;
    std::cout << "Initial right position: " << last_right_position << std::endl;

    // Initialize both arms
    // --------------------
    std::cout << ">>>>>>>>>>>>>>>>> Initialization  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    double z = 0;
    // Left arm initialization
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    // Right arm initialization
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    ArmAction::Ptr target(new ArmWait(1.5));
    left_arm_positions.push(target);
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    
    // First synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> First block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes middle
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    // Left arm turns its pump on, and goes down to take a genoese
    addPumpToLeftQueue(true);
    z = arm::GROUND_HEIGHT + getPileHeight(2) * arm::LAYER_HEIGHT + 5e-3;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    z = arm::GROUND_HEIGHT + getPileHeight(2) * arm::LAYER_HEIGHT - 7e-3;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    changePileHeight(PILE_IDX::MIDDLE, -1);
    target.reset(new ArmWait(1.0));
    left_arm_positions.push(target);
    // Left arm goes up
    z = arm::PILE_CLEAR_HEIGHT + 8e-3;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    
    // Second synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Second block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes to side pile and delivers its genoese
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    z = arm::PILE_CLEAR_HEIGHT + 12e-3;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::LEFT_SIDE) * arm::LAYER_HEIGHT + 2e-2;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    target.reset(new ArmPump(false));
    left_arm_positions.push(target);
    target.reset(new ArmWait(1.0));
    left_arm_positions.push(target);
    z = arm::PILE_CLEAR_HEIGHT + 12e-3;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    // Right arm goes to center, turns its pump on and takes its genoese
    ArmPosition position_tmp(arm::CAKES_FRONT_DISTANCE-5e-3, -30*arm::RAD, z = arm::PILE_CLEAR_HEIGHT + 12e-3);
    addPositionToQueue_Right(position_tmp);
    target.reset(new ArmPump(true));
    right_arm_positions.push(target);
    position_tmp.z_ = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT + 5e-3;
    addPositionToQueue_Right(position_tmp);
    position_tmp.z_ = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT - 7e-3;
    addPositionToQueue_Right(position_tmp);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    target.reset(new ArmWait(1.0));
    right_arm_positions.push(target);
    // Right arm goes to side pile and delivers its genoese
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    position_tmp.z_ = arm::PILE_CLEAR_HEIGHT + 17e-3;
    addPositionToQueue_Right(position_tmp);
    position_tmp.theta_ = sidePile.theta_;
    addPositionToQueue_Right(position_tmp);
    position_tmp.z_ = arm::PILE_CLEAR_HEIGHT + 17e-3;
    position_tmp.theta_ = sidePile.theta_;
    position_tmp.r_ += 5e-3;
    addPositionToQueue_Right(position_tmp);
    position_tmp.z_ = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_SIDE) * arm::LAYER_HEIGHT + 2e-2;
    addPositionToQueue_Right(position_tmp);
    target.reset(new ArmWait(0.2));
    target.reset(new ArmPump(false));
    right_arm_positions.push(target);
    target.reset(new ArmWait(1.0));
    right_arm_positions.push(target);
    position_tmp.z_ = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(position_tmp);
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;
    
    // Third synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Third block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm goes to front pile and takes a cream, and delivers it on the middle pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    //~ target.reset(new ArmPump(true));
    left_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::LEFT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    //~ target.reset(new ArmPump(false));
    //~ left_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    // Left returns to the left front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;
    
    // Fourth synchronize block
    // ------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fourth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes a cream from the front pile
    //~ target.reset(new ArmPump(true));
    //~ left_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::LEFT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    // Left arm delivers the cream to the side pile
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::LEFT_SIDE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    //~ target.reset(new ArmPump(false));
    //~ left_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    // Left arm goes over its front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Right arm takes the cream from the middle pile 
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    //~ target.reset(new ArmPump(true));
    //~ right_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    // Right arm delivers it to the side pile
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_SIDE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    //~ target.reset(new ArmPump(false));
    //~ right_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    // Right arm goes over its front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;
    
    // Fifth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Fifth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm moves cream from its front pile to the middle pile
    //~ target.reset(new ArmPump(true));
    left_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::LEFT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    changePileHeight(PILE_IDX::LEFT_FRONT, -1);
    // Left arm goes to the middle pile and delivers the cream
    changePileHeight(PILE_IDX::MIDDLE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    //~ target.reset(new ArmPump(false));
    //~ left_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    // Left arm goes over the front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Right arm picks the ganache from its front pile
    //~ target.reset(new ArmPump(true));
    //~ right_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    changePileHeight(PILE_IDX::RIGHT_FRONT, -1);
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;

    // Sixth synchronize block
    // -----------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sixth block  <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Right arm delivers ganache to the middle pile
    changePileHeight(PILE_IDX::MIDDLE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    //~ target.reset(new ArmPump(false));
    //~ right_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    // Right arm returns over its front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;
    
    // Seventh synchronize block
    // -------------------------
    std::cout << ">>>>>>>>>>>>>>>>> Sventh block <<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // Left arm takes the ganache from the middle pile
    //~ target.reset(new ArmPump(true));
    left_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    // Left arm delivers the ganache to the side pile
    changePileHeight(PILE_IDX::LEFT_SIDE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    //~ target.reset(new ArmPump(false));
    //~ left_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    left_arm_positions.push(target);
    // Right arm moves ganache from front pile to side pile
    //~ target.reset(new ArmPump(true));
    //~ right_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    changePileHeight(PILE_IDX::RIGHT_SIDE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_SIDE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(sidePile, z));
    //~ target.reset(new ArmPump(false));
    //~ right_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    // Right arm moves over front pile
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Right arm moves ganache from front pile to side pile
    //~ target.reset(new ArmPump(true));
    //~ right_arm_positions.push(target);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::RIGHT_FRONT) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    changePileHeight(PILE_IDX::MIDDLE, -1);
    changePileHeight(PILE_IDX::MIDDLE, 1);
    z = arm::GROUND_HEIGHT + getPileHeight(PILE_IDX::MIDDLE) * arm::LAYER_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(middlePile, z));
    //~ target.reset(new ArmPump(false));
    //~ right_arm_positions.push(target);
    target.reset(new ArmWait(0.5));
    right_arm_positions.push(target);
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
    std::cout << "FINISHED" << std::endl;
    while(true);;

    // Initialize both arms
    // --------------------
    // Left arm initialization
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Left(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Right arm initialization
    z = arm::PILE_CLEAR_HEIGHT;
    addPositionToQueue_Right(ArmPosition::initPositionFromReferenceAndZ(frontPile, z));
    // Synchronize
    addSyncToQueue();
    waitForArmMotionSequenced();
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
