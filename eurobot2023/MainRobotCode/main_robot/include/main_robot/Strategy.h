/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_STRATEGY_H
#define MAIN_ROBOT_STRATEGY_H

#include "common/RobotInterface.h"
#include "common/ServoHandler.h"
#include "common/AbstractAction.h"
#include "common/AbstractStrategy.h"
#include "common/MotionPlanner.h"
#include "main_robot/ArmAction.h"

#include <queue>
#include <mutex>
#include <array>

namespace main_robot
{
    

    // inline std::ostream& operator<<(std::ostream &s, const ArmPosition &armPosition)
    // {
    //     return s << "[" << armPosition.r_ << ", " << armPosition.theta_ << ", " << armPosition.z_ << "]";
    // }

    namespace arm{
        double const GROUND_HEIGHT = -0.195;

        // Position of the cakes
        double const CAKES_FRONT_DISTANCE = 0.115;
        double const CAKES_SIDE_DISTANCE = 0.060;
        double const FRONT_RIGHT_ANGLE = -0.27;
        double const FRONT_LEFT_ANGLE = 0.27;
        double const SIDE_ANGLE = 1.2;

        double const PILE_CLEAR_HEIGHT = GROUND_HEIGHT + 0.085;
        static ArmPosition DISTRIBUTOR_CHERRY(125, -0.55, -130);

        double const LAYER_HEIGHT = 0.02;
        double const LAYER_MOVEMENT_CLEARANCE = 0.025;

        ArmPosition servoAnglesToArmPosition(double thetaHorizontal, double theta12, double theta23, double theta34);
    }

    static int const RIGHT_ARM = 10;
    static int const LEFT_ARM = 20;

    static int const PUMP_RIGHT = 12;
    static int const PUMP_LEFT = 13;
    static int const VALVE_RIGHT = 24;
    static int const VALVE_LEFT = 26;


    class Strategy : public AbstractStrategy
    {
    public:
        // Constructor
        Strategy();

        // Called before the start of the match, to setup the robot.
        void setup(RobotInterface *robot) override;

        // Code executed when shutting down the robot
        void shutdown() override;

        // The actual match code, which runs in its own thread.
        void match() override;

        bool moveArm(double r, double angle, double z);

    private:

        std::queue<std::shared_ptr<ArmAction > > left_arm_positions;
        std::queue<std::shared_ptr<ArmAction > > right_arm_positions;
        std::mutex pileLock;
        std::array<int, 5> pileHeight;

        ArmPosition last_left_position;
        ArmPosition last_right_position;

        void addPositionToQueue_Right(ArmPosition target);
        void addPositionToQueue_Left(ArmPosition target);
        void addSyncToQueue();
        void changePileHeight(int pileIndex, int delta);
        int getPileHeight(int pileIndex);
        void addPumpToLeftQueue(bool activated);
        void addPumpToRightQueue(bool activated);

        void match_impl(); /// Actual implementation of the match code.

        STSServoDriver *servo;

        Action *chooseNextAction(
            std::vector<Action> &actions,
            RobotPosition currentPosition,
            MotionPlanner &motionPlanner);

        /// @brief  \brief Blocks until arms have finished moving
        ArmPosition getArmPosition(int const& armFirstServoId);
        void waitForArmMotion();
        void waitForArmMotionSequenced();
        void depileArm(std::queue<std::shared_ptr<ArmAction > >& actions, int armServoId);
        std::vector<std::shared_ptr<ArmPosition > > computeSequenceToPosition(int const& armFirstServoId, ArmPosition& destination);

        /// @brief  Try to move an arm to a set position, returns false if could not be computed.
        /// @param[in] armPosition Target arm position
        /// @param[in] armFirstServoId Id of the first servo of the arm
        /// @return True if compuation succeeded
    bool setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition);

        /// @brief Execute the cake building sequence
        void buildCakes();
    };


    std::ostream& operator<<(std::ostream& os, const main_robot::ArmPosition& p);
}



#endif
