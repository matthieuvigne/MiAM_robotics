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

    bool Strategy::setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition)
    {
        std::array<double,4> armAngles;
        bool result = common::arm_inverse_kinematics(armPosition.r_, armPosition.theta_, armPosition.z_, -M_PI_2, &armAngles);
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


        ArmPosition leftPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_LEFT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition rightPile(arm::CAKES_FRONT_DISTANCE, arm::FRONT_RIGHT_ANGLE, arm::GROUND_HEIGHT + 0.060);
        ArmPosition sidePile(arm::CAKES_SIDE_DISTANCE, arm::SIDE_ANGLE, arm::GROUND_HEIGHT + 0.020 + 0.020);

        // Grab first item on right pile
        ArmPosition targetPosition = rightPile;
        targetPosition.z_ += 0.025;
        std::cout << "set position" << std::endl;
        // while(true)
        //     std::cout << servo->getCurrentPosition(LEFT_ARM) << std::endl;

        setArmPosition(LEFT_ARM, targetPosition);

        waitForArmMotion();
        RPi_writeGPIO(PUMP_LEFT, true);
        RPi_writeGPIO(VALVE_LEFT, false);
        targetPosition.z_ -= 0.025;
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

    }
}
