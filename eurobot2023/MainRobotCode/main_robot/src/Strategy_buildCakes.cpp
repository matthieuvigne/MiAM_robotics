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
    pileLock.lock();
    pileHeight[pileIndex] += delta;
    pileLock.unlock();

    if (pileHeight[pileIndex] < 0)
        std::cout << ">>>>> WARNING pileHeight[pileIndex] is <0 <<<<<" << std::endl;
}

int Strategy::getPileHeight(int pileIndex)
{
    int height = pileHeight[pileIndex];
    std::cout << "Pile state is " << pileHeight[0] << " " << pileHeight[1] << " " << pileHeight[2] << " " << pileHeight[3] << " " << pileHeight[4] << std::endl;
    return height;
}

void Strategy::addPositionToQueue_Left(ArmPosition target)
{
    // std::cout << "Request position: " << std::endl;
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
    ArmPosition leftPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_LEFT_ANGLE, arm::GROUND_HEIGHT + 0.060);
    ArmPosition rightPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_RIGHT_ANGLE, arm::GROUND_HEIGHT + 0.060);
    ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_ANGLE, arm::GROUND_HEIGHT + 0.020 + 0.020);

    // reset piles and arm positions

    pileHeight[0] = 0;
    pileHeight[1] = 3;
    pileHeight[2] = 3;
    pileHeight[3] = 3;
    pileHeight[4] = 0;

    last_left_position = getArmPosition(LEFT_ARM);
    last_right_position = getArmPosition(RIGHT_ARM);

    std::cout << "Initial left position: " << last_left_position << std::endl;
    std::cout << "Initial right position: " << last_right_position << std::endl;

    // bras gauche va au milieu 
    addPositionToQueue_Left(
        ArmPosition::initPositionFromReferenceAndZ(rightPile, arm::PILE_CLEAR_HEIGHT)
    );

    // bras gauche allume sa pompe
    addPumpToLeftQueue(true);

    // bras gauche descend
    // bras gauche prend une genoise
    {
        
        ArmPosition target = rightPile;
        target.z_ = arm::GROUND_HEIGHT + getPileHeight(2) * arm::LAYER_HEIGHT;
        addPositionToQueue_Left(target);
        changePileHeight(3, -1);
    }

    {
        
        ArmWait::Ptr target(new ArmWait(0.5));
        left_arm_positions.push(target);
    }

    // bras gauche remonte
    {
        
        ArmPosition target = rightPile;
        target.z_ = arm::PILE_CLEAR_HEIGHT;
        addPositionToQueue_Left(target);
    }

    // SYNC   
    addSyncToQueue();


    waitForArmMotionSequenced();
    // while(true) ;;

    // bras gauche va side pile et depose sa genoise
    {
        
        changePileHeight(0, 1);
        ArmPosition target = sidePile;
        target.z_ = arm::GROUND_HEIGHT + getPileHeight(0) * arm::LAYER_HEIGHT;
        addPositionToQueue_Left(target);
    }
    {
        
        ArmPump::Ptr target(new ArmPump(false));
        left_arm_positions.push(target);
    }
    {
        
        ArmWait::Ptr target(new ArmWait(0.5));
        left_arm_positions.push(target);
    }
    {
        
        ArmPosition target = sidePile;
        target.z_ = arm::PILE_CLEAR_HEIGHT;
        addPositionToQueue_Left(target);
    }

    // bras droit va au cemtre et va prendre sa genoise
    {
        
        ArmPosition target = rightPile;
        target.z_ = arm::PILE_CLEAR_HEIGHT;
        addPositionToQueue_Right(target);
    }
    // bras droit allume sa pompe
    {
        
        ArmPump::Ptr target(new ArmPump(true));
        right_arm_positions.push(target);
    }
    {
        
        ArmPosition target = rightPile;
        target.z_ = arm::GROUND_HEIGHT + getPileHeight(3) * arm::LAYER_HEIGHT;
        addPositionToQueue_Right(target);
        changePileHeight(3, -1);
    }
    {
        
        ArmWait::Ptr target(new ArmWait(0.5));
        left_arm_positions.push(target);
    }
    {
        
        changePileHeight(4, 1);
        ArmPosition target = sidePile;
        target.z_ = arm::GROUND_HEIGHT + getPileHeight(4) * arm::LAYER_HEIGHT;
        addPositionToQueue_Right(target);
    }
    {
        
        ArmPump::Ptr target(new ArmPump(false));
        right_arm_positions.push(target);
    }
    {
        
        ArmWait::Ptr target(new ArmWait(0.5));
        left_arm_positions.push(target);
    }
    {
        
        ArmPosition target = sidePile;
        target.z_ = arm::PILE_CLEAR_HEIGHT;
        addPositionToQueue_Right(target);
    }

    // SYNC
    addSyncToQueue();

    waitForArmMotionSequenced();

    while(true) ;;

    // bras droit remonte et va side pile et depose
    // bras gauche va pile du milieu et prend sa creme
    // bras gauche remonte et pose la creme au milieu
    // bras gauche remonte et va au dessus de la pile de gauche
    // SYNC

    // bras gauche descend et prend la creme
    // bras gauche le pose sur side pile
    // bras droit va au milieu et prend la creme
    // bras droit le pose sur sa side pile
    // SYNC

    // bras gauche prend creme pile de gauche
    // bras gauche l'emmene sur pile du centre
    // bras droit va pile de droite
    // bras droit prend la ganache
    // bras droit l'emmene sur sa side pile
    // SYNC

    // bras gauche va a gauche
    // bras droit va a droite
    // bras droit prend la ganache et l'emmene au milieu
    // bras droit retourne sur pile de droite
    // SYNC

    // bras gauche va chercher la ganache au milieu
    // bras gauche l'emmene au dessus de sa side pile
    // SYNC

    // bras gauche la pose sur sa side pile
    // bras droit prend la ganache sur la pile de droite
    // bras droit la pose sur la pile du milieu
    // END












    // std::vector<std::shared_ptr<ArmPosition > > seq = computeSequenceToPosition(LEFT_ARM, sidePile);
    // for (auto& ap : seq)
    // {
    //     left_arm_positions.push(ap);
    // }

    // waitForArmMotionSequenced();

    // while(true) ;;



    // std::cout << "leftPile: " << leftPile.r_ << " " << leftPile.theta_ << " " << leftPile.z_ << std::endl;


    // std::array<double, 4 > destArray;
    // common::arm_inverse_kinematics(leftPile.r_, leftPile.theta_, leftPile.z_, -M_PI_2, &destArray);
    // ArmPosition leftPile2 = servoAnglesToArmPosition(destArray[0], destArray[1], destArray[2], destArray[3]);
    // std::cout << "leftPile2: " << leftPile2.r_ << " " << leftPile2.theta_ << " " << leftPile2.z_ << std::endl;

    // // left arm
    // {
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(leftPile));
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
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(rightPile));
    //     targetPosition->z_ += LAYER_MOVEMENT_CLEARANCE;
    //     left_arm_positions.push (targetPosition);
    // }

    // {
    //     std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(rightPile));
    //     left_arm_positions.push (targetPosition);
    // }

    // waitForArmMotionSequenced();

    // std::cout << "Sequence finished" << std::endl;

    return;





    // go above right pile
    ArmPosition targetPosition = rightPile;
    targetPosition.z_ += LAYER_MOVEMENT_CLEARANCE;
    setArmPosition(LEFT_ARM, targetPosition);
    waitForArmMotion();

    // grab
    targetPosition.z_ -= LAYER_MOVEMENT_CLEARANCE;
    setArmPosition(LEFT_ARM, targetPosition);
    robot->wait(1.5);
    targetPosition.z_ = arm::PILE_CLEAR_HEIGHT;
    setArmPosition(LEFT_ARM, targetPosition);
    waitForArmMotion();




    targetPosition.theta_ = sidePile.theta_;
    setArmPosition(LEFT_ARM, targetPosition);
    waitForArmMotion();
    targetPosition.z_ = arm::GROUND_HEIGHT + 0.035;
    setArmPosition(LEFT_ARM, targetPosition);
    waitForArmMotion();
    RPi_writeGPIO(PUMP_LEFT, false);
    RPi_writeGPIO(VALVE_LEFT, true);
    robot->wait(0.1);
    RPi_writeGPIO(VALVE_LEFT, false);



    // // Grab first item on right pile with left arm
    // ArmPosition targetPosition = rightPile;
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
