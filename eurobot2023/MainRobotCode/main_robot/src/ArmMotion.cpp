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
                case ActionType::PUMP :
                {
                    std::cout << "Pump" << std::endl;
                    std::shared_ptr<ArmPump> action(dynamic_cast<ArmPump*>(actions.front().get()));

                    int pumpNumber  = (armServoId == RIGHT_ARM) ? PUMP_RIGHT : PUMP_LEFT;
                    int valveNumber = (armServoId == RIGHT_ARM) ? VALVE_RIGHT : VALVE_LEFT;
                    RPi_writeGPIO(pumpNumber, action->activated_);
                    RPi_writeGPIO(valveNumber, !action->activated_);
                    actions.pop();
                    break;
                }
                case ActionType::MOVE :
                {

                    std::cout << "Move" << std::endl;
                    ArmPosition* action = dynamic_cast<ArmPosition*>(actions.front().get());

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
        // TODO dans le cas ou on est sur la bonne pile, descendre directement

        // sinon
        ArmPosition currentArmPosition = getArmPosition(armFirstServoId);
        currentArmPosition.z_ = PILE_CLEAR_HEIGHT;

        std::shared_ptr<ArmPosition > moveUp(new ArmPosition(currentArmPosition));
        res.push_back(moveUp);

        currentArmPosition.theta_ = destination.theta_;
        std::shared_ptr<ArmPosition > moveSide(new ArmPosition(currentArmPosition));
        res.push_back(moveSide);

        std::shared_ptr<ArmPosition > moveDown(new ArmPosition(destination));
        res.push_back(moveDown);

        return res;
    }

    ArmPosition Strategy::getArmPosition(int const& armFirstServoId)
    {
        
        // lire les 4 angles
        double thetaHorizontal = STS::servoToRadValue(servo->getCurrentPosition(armFirstServoId + 0));
        double theta12 = STS::servoToRadValue(servo->getCurrentPosition(armFirstServoId + 1));
        double theta23 = STS::servoToRadValue(servo->getCurrentPosition(armFirstServoId + 2));
        double theta34 = STS::servoToRadValue(servo->getCurrentPosition(armFirstServoId + 3));

        if (armFirstServoId == RIGHT_ARM)
            thetaHorizontal = -thetaHorizontal;
        
        return servoAnglesToArmPosition(thetaHorizontal, theta12, theta23, theta34);
    }

    bool Strategy::setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition)
    {
        std::array<double,4> armAngles;

        // invert theta if right arm
        double targetTheta = armPosition.theta_;
        if (armFirstServoId == RIGHT_ARM)
        {
            targetTheta = -targetTheta;
        }

        bool result = common::arm_inverse_kinematics(armPosition.r_, targetTheta, armPosition.z_, -M_PI_2, &armAngles);
        if (result)
        {
            // move servo
            for (int i = 0; i < 4; i++)
            {
                double angle = armAngles[i];
                std::cout << angle << " ";
                if (i == 0)
                    angle = -angle;
                servo->setTargetPosition(armFirstServoId + i, STS::radToServoValue(angle));
                usleep(50);
            }

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

    void Strategy::buildCakes()
    {
        std::cout << "Building cakes" << std::endl;
        
        // right arm has inverted angles!
        ArmPosition leftPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_LEFT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition rightPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_RIGHT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_ANGLE, arm::GROUND_HEIGHT + 0.020 + 0.020);


        std::vector<std::shared_ptr<ArmPosition > > seq = computeSequenceToPosition(LEFT_ARM, sidePile);
        for (auto& ap : seq)
        {
            left_arm_positions.push(ap);
        }

        waitForArmMotionSequenced();

        while(true) ;;



        std::cout << "leftPile: " << leftPile.r_ << " " << leftPile.theta_ << " " << leftPile.z_ << std::endl;


        std::array<double, 4 > destArray;
        common::arm_inverse_kinematics(leftPile.r_, leftPile.theta_, leftPile.z_, -M_PI_2, &destArray);
        ArmPosition leftPile2 = servoAnglesToArmPosition(destArray[0], destArray[1], destArray[2], destArray[3]);
        std::cout << "leftPile2: " << leftPile2.r_ << " " << leftPile2.theta_ << " " << leftPile2.z_ << std::endl;

        // left arm
        {
            std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(leftPile));
            left_arm_positions.push (targetPosition);
        }

        waitForArmMotionSequenced();

        ArmPosition leftArm = getArmPosition(LEFT_ARM);
        std::cout << "leftArm: " << leftArm.r_ << " " << leftArm.theta_ << " " << leftArm.z_ << std::endl;

        ArmPosition rightArm = getArmPosition(RIGHT_ARM);
        std::cout << "rightArm: " << rightArm.r_ << " " << rightArm.theta_ << " " << rightArm.z_ << std::endl;

        while(true) ;;

        // Grab first item on right pile with left arm

        // left arm
        {
            std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(rightPile));
            targetPosition->z_ += LAYER_MOVEMENT_CLEARANCE;
            left_arm_positions.push (targetPosition);
        }

        {
            std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(rightPile));
            left_arm_positions.push (targetPosition);
        }

        waitForArmMotionSequenced();

        std::cout << "Sequence finished" << std::endl;

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
