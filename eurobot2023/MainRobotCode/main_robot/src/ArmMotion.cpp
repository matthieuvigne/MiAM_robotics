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
    std::ostream& operator<<(std::ostream& os, const ArmPosition& p)
    {
        os << "r: " << p.r_ << " theta(deg): " << 180 * p.theta_ / M_PI << " z: " << p.z_;
        return os;
    }

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

        // std::cout << "Depiling " << (armServoId == RIGHT_ARM ? "right" : "left") <<  " arm " << std::endl;

        std::string armName((armServoId == RIGHT_ARM ? "right" : "left"));

        while(!actions.empty())
        {
            // std::cout << "Actions not empty: size " << actions.size() << std::endl;
            switch(actions.front()->type_)
            {
                // if sync, pop and end
                case ActionType::SYNC :
                {
                    std::cout << "### Sync " << armName << std::endl;
                    actions.pop();
                    return;
                }
                case ActionType::WAIT :
                {

                    ArmWait* action(dynamic_cast<ArmWait*>(actions.front().get()));

                    std::cout << ">>> Wait " << armName << " t=" << action->time_ << std::endl;
                    usleep(uint(1e6 * action->time_));
                    actions.pop();
                    return;
                }
                case ActionType::PUMP :
                {
                    ArmPump* action(dynamic_cast<ArmPump*>(actions.front().get()));
                    std::cout << ">>> Pump " << armName << " " << action->activated_ << std::endl;

                    int pumpNumber  = (armServoId == RIGHT_ARM) ? PUMP_RIGHT : PUMP_LEFT;
                    int valveNumber = (armServoId == RIGHT_ARM) ? VALVE_RIGHT : VALVE_LEFT;
                    RPi_writeGPIO(pumpNumber, action->activated_);
                    RPi_writeGPIO(valveNumber, !action->activated_);
                    actions.pop();
                    break;
                }
                case ActionType::MOVE :
                {
                    ArmPosition* action = dynamic_cast<ArmPosition*>(actions.front().get());
                    std::cout << ">>>> Move " << armName << " " << *action << std::endl;

                    if (action->z_ > 0)
                        std::cout << ">>>>> WARNING : z is positive <<<<<" << std::endl;
                    setArmPosition(armServoId, *action);
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
                    actions.pop();
                    break;
                }
            }
        }
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
    }

    std::vector<std::shared_ptr<ArmPosition > > Strategy::computeSequenceToPosition(int const& armFirstServoId, ArmPosition& destination)
    {
        std::vector<std::shared_ptr<ArmPosition > > res;
        ArmPosition currentArmPosition;
        if (armFirstServoId == LEFT_ARM)
        {
            currentArmPosition = last_left_position;
        }
        else
        {
            currentArmPosition = last_right_position;
        }
        std::cout << "Current position: " << currentArmPosition << " - destination: " << destination << std::endl;

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

        if (armFirstServoId == LEFT_ARM)
        {
            last_left_position = *moveDown;
        }
        else
        {
            last_right_position = *moveDown;
        }

        return res;
    }



    ArmPosition Strategy::getArmPosition(int const& armFirstServoId)
    {
        
        // lire les 4 angles
        double thetaHorizontal = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 0));
        double theta12 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 1));
        double theta23 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 2));
        double theta34 = STS::servoToRadValue(servo->getLastCommand(armFirstServoId + 3));

        std::cout << "Read servo " << armFirstServoId << " : " << thetaHorizontal << " " << theta12 << " " << theta23 << " " << theta34 << std::endl; 

        // if (armFirstServoId == RIGHT_ARM)
        //     thetaHorizontal = -thetaHorizontal;
        
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
                if (armFirstServoId == RIGHT_ARM)
                    angle = -angle;
                servo->setTargetPosition(armFirstServoId + i, STS::radToServoValue(angle));
                usleep(50);
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

    void Strategy::addPositionToQueue_Left(ArmPosition& target)
    {
        // std::cout << "Request position: " << std::endl;
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
}