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

#include <queue>

namespace main_robot
{

    enum ActionType
    {
        SYNC = 0,
        MOVE = 1,
        PUMP = 2
    };

    class ArmAction
    {
    public:
        ArmAction() {}
        virtual ~ArmAction() = default;
        ActionType type_;
    };

    class ArmSync : public ArmAction
    {
        ArmSync() : ArmAction() 
        {
            type_ = ActionType::SYNC;
        };
    };

    class ArmPosition : public ArmAction
    {
    public:
        double r_;
        double theta_;
        double z_;

        ArmPosition(double r, double theta, double z) : ArmAction(), r_(r), theta_(theta), z_(z)
        {
            type_ = ActionType::MOVE;
        };
    };

    class ArmPump : public ArmAction
    {
    public:
        bool activated_;

        ArmPump(double activated) : ArmAction(), activated_(activated) {
            type_ = ActionType::PUMP;
        };
    };

    inline std::ostream& operator<<(std::ostream &s, const ArmPosition &armPosition)
    {
        return s << "[" << armPosition.r_ << ", " << armPosition.theta_ << ", " << armPosition.z_ << "]";
    }

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

        void match_impl(); /// Actual implementation of the match code.

        STSServoDriver *servo;

        Action *chooseNextAction(
            std::vector<Action> &actions,
            RobotPosition currentPosition,
            MotionPlanner &motionPlanner);

        /// @brief  \brief Blocks until arms have finished moving
        void waitForArmMotion();
        void waitForArmMotionSequenced();
        void depileArm(std::queue<std::shared_ptr<ArmAction > >& actions, int armServoId);

        /// @brief  Try to move an arm to a set position, returns false if could not be computed.
        /// @param[in] armPosition Target arm position
        /// @param[in] armFirstServoId Id of the first servo of the arm
        /// @return True if compuation succeeded
    bool setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition);

        /// @brief Execute the cake building sequence
        void buildCakes();
    };
}

#endif
