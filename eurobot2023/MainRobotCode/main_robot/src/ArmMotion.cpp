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
                    break;
                }
                case ActionType::MOVE :
                {

                    std::cout << "Move" << std::endl;
                    std::shared_ptr<ArmPosition> action(dynamic_cast<ArmPosition*>(actions.front().get()));

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
                    break;
                }
                actions.pop();
            }
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


        // Grab first item on right pile with left arm

        // left arm
        {
            std::shared_ptr<ArmPosition > targetPosition(new ArmPosition(rightPile));
            targetPosition->z_ += LAYER_MOVEMENT_CLEARANCE;
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
