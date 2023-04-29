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

using namespace main_robot::arm;

std::ostream& operator<<(std::ostream& os, const main_robot::ArmPosition& p)
{
    os << "r: " << p.r_ << " theta(deg): " << 180 * p.theta_ / M_PI << " z: " << p.z_;
    return os;
}

namespace main_robot
{
    ArmPosition arm::servoAnglesToArmPosition(double thetaHorizontal, double theta12, double theta23, double theta34)
    {
        // Arm parameters
        double constexpr d12 = 76.5e-3;
        double constexpr d23 = 70.6e-3;
        double constexpr d34 = 90.6e-3;
        double constexpr d45x = 29.1e-3;
        double constexpr d45z = 45.8e-3;

        double r = d12 
            + d23 * std::cos(theta12) 
            + d34 * std::cos(theta12 + theta23) 
            + d45x * std::cos(theta12 + theta23 + theta34)
            - d45z * std::sin(theta12 + theta23 + theta34);

        double z = d23 * std::sin(theta12)
            + d34 * std::sin(theta12 + theta23)
            + d45x * std::sin(theta12 + theta23 + theta34)
            + d45z * std::cos(theta12 + theta23 + theta34);


        return ArmPosition(r, thetaHorizontal, z);
    }

    void Strategy::waitForArmMotion()
    {
        usleep(500000);
        // return;
        bool done = false;
        while (!done)
        {
            done = true;
            for (int i = 0; i < 4; i++)
            {
                done &= !servo->isMoving(RIGHT_ARM + i);
                done &= !servo->isMoving(LEFT_ARM + i);
            }
            usleep(20000);
        }
    }

    void Strategy::depileArm(std::queue<std::shared_ptr<ArmAction > >& actions, int armServoId)
    {

        std::cout << "Depiling " << (armServoId == RIGHT_ARM ? "right" : "left") <<  " arm " << std::endl;
        while(!actions.empty())
        {
            std::cout << "Actions not empty: size " << actions.size() << std::endl;
            switch(actions.front()->type_)
            {
                // if sync, pop and end
                case ActionType::SYNC :
                {
                    std::cout << "Sync" << std::endl;
                    actions.pop();
                    return;
                }
                case ActionType::WAIT :
                {
                    std::cout << "Wait" << std::endl;

                    ArmWait* action(dynamic_cast<ArmWait*>(actions.front().get()));
                    usleep(uint(1e6 * action->time_));
                    actions.pop();
                    return;
                }
                case ActionType::PUMP :
                {
                    std::cout << "Pump" << std::endl;
                    ArmPump* action(dynamic_cast<ArmPump*>(actions.front().get()));

                    int pumpNumber  = (armServoId == RIGHT_ARM) ? PUMP_RIGHT : PUMP_LEFT;
                    int valveNumber = (armServoId == RIGHT_ARM) ? VALVE_RIGHT : VALVE_LEFT;
                    RPi_writeGPIO(pumpNumber, action->activated_);
                    RPi_writeGPIO(valveNumber, !action->activated_);
                    std::cout << "End Pump" << std::endl;
                    actions.pop();
                    std::cout << "Pop action" << std::endl;
                    break;
                }
                case ActionType::MOVE :
                {

                    std::cout << "Move" << std::endl;
                    ArmPosition* action = dynamic_cast<ArmPosition*>(actions.front().get());

                    if (action->z_ > 0)
                        std::cout << ">>>>> WARNING : z is positive <<<<<" << std::endl;
                    setArmPosition(armServoId, *action);

                    std::cout << "I set the arm to move" << std::endl;
                    usleep(500000);
                    // return;
                    bool done = false;
                    while (!done)
                    {
                        done = true;
                        for (int i = 0; i < 4; i++)
                        {
                            done &= !servo->isMoving(armServoId + i);
                        }
                        usleep(20000);
                    }  

                    std::cout << "Move finished" << std::endl;
                    std::cout << "Action length: " << actions.size() << std::endl;
                    actions.pop();
                    break;
                }
            }
            std::cout << "end of while" << std::endl;
        }


        std::cout << "end of depileArm" << std::endl;
    }

    void Strategy::waitForArmMotionSequenced()
    {

        while (!right_arm_positions.empty() | !left_arm_positions.empty())
        {

            std::cout << "SequenceTicked" << std::endl;

            std::thread thread_right([this](){depileArm(right_arm_positions, RIGHT_ARM);});
            std::thread thread_left([this](){depileArm(left_arm_positions, LEFT_ARM);});

            thread_right.join();
            thread_left.join();

            std::cout << "Threads joined" << std::endl;

        }


        std::cout << "end of waitForArmMotionSequenced" << std::endl;

    }

    std::vector<std::shared_ptr<ArmPosition > > Strategy::computeSequenceToPosition(int const& armFirstServoId, ArmPosition& destination)
    {
        std::vector<std::shared_ptr<ArmPosition > > res;
        ArmPosition currentArmPosition = getArmPosition(armFirstServoId);
        // dans le cas ou on est sur la bonne pile, descendre directement
        if (abs(moduloTwoPi(currentArmPosition.theta_ - destination.theta_)) > 5.0 * M_PI / 180.0)
        {
            currentArmPosition.z_ = PILE_CLEAR_HEIGHT;

            std::shared_ptr<ArmPosition > moveUp(new ArmPosition(currentArmPosition));
            res.push_back(moveUp);

            currentArmPosition.theta_ = destination.theta_;
            std::shared_ptr<ArmPosition > moveSide(new ArmPosition(currentArmPosition));
            res.push_back(moveSide);
        }

        std::shared_ptr<ArmPosition > moveDown(new ArmPosition(destination));
        res.push_back(moveDown);

        return res;
    }



    ArmPosition Strategy::getArmPosition(int const& armFirstServoId)
    {
        
        // lire les 4 angles
        double thetaHorizontal = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 0));
        double theta12 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 1));
        double theta23 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 2));
        double theta34 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 3));

        if (armFirstServoId == RIGHT_ARM)
            thetaHorizontal = -thetaHorizontal;
        
        return servoAnglesToArmPosition(thetaHorizontal, theta12, theta23, theta34);
    }

    bool Strategy::setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition)
    {
        std::array<double,4> armAngles;

        // invert theta if right arm
        double targetTheta = armPosition.theta_;
        // if (armFirstServoId == RIGHT_ARM)
        // {
        //     targetTheta = -targetTheta;
        // }

        std::cout << "Target: " << armPosition << std::endl;
        bool result = common::arm_inverse_kinematics(armPosition.r_, targetTheta, armPosition.z_, -M_PI_2, &armAngles);

        // TODO
        if(!result)
        {
            std::cout << "Inverse kinematics failed" << std::endl;
            throw std::runtime_error("AAA");
        }

        // TODO
        if(armAngles[1] > 0)
        {
            std::cout << "1st Angle is too high: " << armAngles[1] << std::endl;
            throw std::runtime_error("AAA");
        }

        if (result)
        {
            // move servo
            std::cout << "angles: ";
            for (int i = 0; i < 4; i++)
            {
                double angle = armAngles[i];
                std::cout << angle << " ";
                if (armFirstServoId == RIGHT_ARM)
                    angle = -angle;
                servo->setTargetPosition(armFirstServoId + i, STS::radToServoValue(angle));
                usleep(50);
            }
            std::cout << std::endl;
            ArmPosition pos = servoAnglesToArmPosition(armAngles[0], armAngles[1], armAngles[2], armAngles[3]);
            std::cout << "What i found: " <<  pos << std::endl;

            // // pump
            // if (armFirstServoId == RIGHT_ARM)
            // {
            //     RPi_writeGPIO(PUMP_RIGHT, armPosition_.pumpActivated_);
            //     RPi_writeGPIO(VALVE_RIGHT, !armPosition_.pumpActivated_);
            // }
            // else
            // {
            //     RPi_writeGPIO(PUMP_LEFT, armPosition_.pumpActivated_);
            //     RPi_writeGPIO(VALVE_LEFT, !armPosition_.pumpActivated_);
            // }
        }
        else
        {
            std::cout << "failed computation" << std::endl;
        }
        std::cout << std::endl;
        return result;
    }

    void Strategy::addPositionToQueue_Left(ArmPosition& target)
    {
        std::cout << "Request position: " << std::endl;
        std::vector<ArmPosition::Ptr > seq = computeSequenceToPosition(LEFT_ARM, target);
        for (auto& ap : seq)
        {
            left_arm_positions.push(ap);
        }
    }

    void Strategy::addPositionToQueue_Right(ArmPosition& target)
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

    void Strategy::buildCakes()
    {
        std::cout << "Building cakes" << std::endl;
        
        // right arm has inverted angles!
        ArmPosition leftPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_LEFT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition rightPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_RIGHT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_ANGLE, arm::GROUND_HEIGHT + 0.020 + 0.020);

        pileHeight[0] = 0;
        pileHeight[1] = 3;
        pileHeight[2] = 3;
        pileHeight[3] = 3;
        pileHeight[4] = 0;

        int moveNumber = 0;

        // bras gauche va au milieu 
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPosition target = rightPile;
            target.z_ = arm::PILE_CLEAR_HEIGHT;
            addPositionToQueue_Left(target);
        }

        // bras gauche allume sa pompe
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPump::Ptr target(new ArmPump(true));
            left_arm_positions.push(target);
        }

        // bras gauche descend
        // bras gauche prend une genoise
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPosition target = rightPile;
            target.z_ = arm::GROUND_HEIGHT + getPileHeight(2) * arm::LAYER_HEIGHT;
            addPositionToQueue_Left(target);
            changePileHeight(3, -1);
        }

        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmWait::Ptr target(new ArmWait(0.5));
            left_arm_positions.push(target);
        }

        // bras gauche remonte
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
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
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            changePileHeight(0, 1);
            ArmPosition target = sidePile;
            target.z_ = arm::GROUND_HEIGHT + getPileHeight(0) * arm::LAYER_HEIGHT;
            addPositionToQueue_Left(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPump::Ptr target(new ArmPump(false));
            left_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmWait::Ptr target(new ArmWait(0.5));
            left_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPosition target = sidePile;
            target.z_ = arm::PILE_CLEAR_HEIGHT;
            addPositionToQueue_Left(target);
        }

        // bras droit va au cemtre et va prendre sa genoise
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPosition target = rightPile;
            target.z_ = arm::PILE_CLEAR_HEIGHT;
            addPositionToQueue_Right(target);
        }
        // bras droit allume sa pompe
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPump::Ptr target(new ArmPump(true));
            right_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPosition target = rightPile;
            target.z_ = arm::GROUND_HEIGHT + getPileHeight(3) * arm::LAYER_HEIGHT;
            addPositionToQueue_Right(target);
            changePileHeight(3, -1);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmWait::Ptr target(new ArmWait(0.5));
            left_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            changePileHeight(4, 1);
            ArmPosition target = sidePile;
            target.z_ = arm::GROUND_HEIGHT + getPileHeight(4) * arm::LAYER_HEIGHT;
            addPositionToQueue_Right(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmPump::Ptr target(new ArmPump(false));
            right_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
            ArmWait::Ptr target(new ArmWait(0.5));
            left_arm_positions.push(target);
        }
        {
            moveNumber++;
            std::cout << "############ MOVE NUMBER: " << moveNumber << " ############" << std::endl;
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
}
