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
  
//--------------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const ArmPosition& p)
{
    os << "r: " << p.r_ << " theta(deg): " << 180 * p.theta_ / M_PI << " z: " << p.z_;
    return os;
}

//--------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------

ArmPosition ArmPosition::initPositionFromReferenceAndZ(ArmPosition const& reference, double z)
{
    ArmPosition res(reference);
    if (!std::isnan(z))
    {
        res.z_ = z;
    }
    return res;
}

//--------------------------------------------------------------------------------------------------

void Strategy::waitForArmMotion()
{
#ifndef SIMULATION
    //~ usleep(500000);
    //~ std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
        //~ usleep(20000);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
#endif
}

//--------------------------------------------------------------------------------------------------

void Strategy::depileArm(std::queue<std::shared_ptr<ArmAction>>& actions, int armServoId)
{
    std::string armName((armServoId == RIGHT_ARM ? "right" : "left"));
    std::cout << ">>>>>> Number of actions in pile of " << armName << " arm: " << actions.size() << std::endl;

    while(!actions.empty())
    {
        std::cout << "New action depiling " << (armServoId == RIGHT_ARM ? "right" : "left") << " arm " << std::endl;
        std::cout << "Remaining actions: " << actions.size() << std::endl;

        // std::cout << "Actions not empty: size " << actions.size() << std::endl;
        switch(actions.front()->type_)
        {
            // if sync, pop and end
            case ActionType::SYNC :
            {
                std::cout << "### Sync " << armName << std::endl;
                actions.pop();
                std::cout << "Popped sync action" << std::endl;
                break;
            }
            case ActionType::WAIT :
            {
                ArmWait* action(dynamic_cast<ArmWait*>(actions.front().get()));
                std::cout << ">>> Wait " << armName << " t=" << action->time_ << std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(uint(1e6*action->time_)));
                std::cout << "Waited" << std::endl;
                actions.pop();
                break;
            }
            case ActionType::PUMP :
            {
                ArmPump* action(dynamic_cast<ArmPump*>(actions.front().get()));
                std::cout << ">>> Pump " << armName << " " << action->activated_ << std::endl;

                int pumpNumber  = (armServoId == RIGHT_ARM) ? PUMP_RIGHT : PUMP_LEFT;
                int valveNumber = (armServoId == RIGHT_ARM) ? VALVE_RIGHT : VALVE_LEFT;
                RPi_writeGPIO(pumpNumber, action->activated_);
                RPi_writeGPIO(valveNumber, !action->activated_);
                if(!action->activated_){
                  std::this_thread::sleep_for(std::chrono::milliseconds(100));
                  RPi_writeGPIO(valveNumber, false);
                }
                actions.pop();
                break;
            }
            case ActionType::MOVE :
            {
                // Check the current during the action
                // double getCurrentCurrent(unsigned char const& servoId);
                //~ std::array<double,4> current; // [TO REMOVE]
              
                ArmPosition* action = dynamic_cast<ArmPosition*>(actions.front().get());
                std::cout << ">>>> Move " << armName << " " << *action << std::endl;

                if (action->z_ > 0)
                    std::cout << ">>>>> WARNING : z is positive <<<<<" << std::endl;
                setArmPosition(armServoId, *action);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                bool done = false;
                while (!done)
                {
                    done = true;
                    for (int i = 0; i < 4; i++)
                    {
                        done &= !servo->isMoving(armServoId + i);
                        //~ current[i] = servo->getCurrentCurrent(armServoId + i); // [TO REMOVE]
                        //~ std::cout << "Servo[" << i << "] " << 100*current[i] << std::endl; // [TO REMOVE]
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(2));
                }
                actions.pop();
                break;
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------

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
}

//--------------------------------------------------------------------------------------------------

std::vector<std::shared_ptr<ArmPosition>> Strategy::computeSequenceToPosition(
  int const& armFirstServoId, ArmPosition& destination)
{
    std::vector<std::shared_ptr<ArmPosition>> res;
    ArmPosition currentArmPosition;
    if (armFirstServoId == LEFT_ARM)
        currentArmPosition = last_left_position;
    else
        currentArmPosition = last_right_position;
    std::cout << "Current position: " << currentArmPosition << " - destination: " << destination << std::endl;
    std::shared_ptr<ArmPosition > moveDown(new ArmPosition(destination));
    res.push_back(moveDown);

    if (armFirstServoId == LEFT_ARM)
        last_left_position = *moveDown;
    else
        last_right_position = *moveDown;

    return res;
}

//--------------------------------------------------------------------------------------------------

ArmPosition Strategy::getArmPosition(int const& armFirstServoId)
{

    // lire les 4 angles
    double thetaHorizontal = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 0));
    double theta12 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 1));
    double theta23 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 2));
    double theta34 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 3));

    std::cout << "Read servo " << armFirstServoId << " : " << thetaHorizontal << " " << theta12 << " " << theta23 << " " << theta34 << std::endl;

    if (armFirstServoId == RIGHT_ARM)
    {
        thetaHorizontal = -thetaHorizontal;
        theta12 = -theta12;
        theta23 = -theta23;
        theta34 = -theta34;
    }

    ArmPosition res(servoAnglesToArmPosition(thetaHorizontal, theta12, theta23, theta34));

    std::cout << "Converted to ArmPosition " << armFirstServoId << " : " << res << std::endl;
    return res;
}

//--------------------------------------------------------------------------------------------------

bool Strategy::setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition)
{
    std::array<double,4> armAngles;

    // invert theta if right arm
    double targetTheta = armPosition.theta_;
    // if (armFirstServoId == RIGHT_ARM)
    // {
    //     targetTheta = -targetTheta;
    // }

    // std::cout << "Target: " << armPosition << std::endl;
    bool result = common::arm_inverse_kinematics(armPosition.r_, targetTheta, armPosition.z_, -M_PI_2, &armAngles);

    // TODO
    if(!result)
    {
        std::cout << "Inverse kinematics failed" << std::endl;
        std::cout << "armAngles: ";
        for (int i=0; i<4; i++)
            std::cout << armAngles[i] << " ";
        std::cout << std::endl;
        throw std::runtime_error("AAA");
    }

    // TODO
    if(armAngles[1] > 0)
    {
        std::cout << "armAngles: ";
        for (int i=0; i<4; i++)
            std::cout << armAngles[i] << " ";
        std::cout << std::endl;
        std::cout << "1st Angle is too high: " << armAngles[1] << std::endl;
        throw std::runtime_error("AAA");
    }

    if (result)
    {
        // move servo
        // std::cout << "angles: ";
        for (int i = 0; i < 4; i++)
        {
            double angle = armAngles[i];
            // std::cout << angle << " ";
            if(i==0 && armFirstServoId == LEFT_ARM)
              angle = -angle;
            if(i>0 && armFirstServoId == RIGHT_ARM)
                angle = -angle;

#ifndef SIMULATION
            servo->setTargetPosition(armFirstServoId + i, STS::radToServoValue(angle));
            //~ usleep(50);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
#endif
        }
        // std::cout << std::endl;
        ArmPosition pos = servoAnglesToArmPosition(armAngles[0], armAngles[1], armAngles[2], armAngles[3]);
        // std::cout << "What i found: " <<  pos << std::endl;

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
    // else
    // {
    //     std::cout << "failed computation" << std::endl;
    // }
    // std::cout << std::endl;
    return result;
}

//--------------------------------------------------------------------------------------------------
}
