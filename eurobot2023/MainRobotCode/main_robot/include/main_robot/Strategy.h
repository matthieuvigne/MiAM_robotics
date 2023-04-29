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

namespace main_robot
{
    class ArmPosition
    {
    public:
        double r_;
        double theta_;
        double z_;

        ArmPosition(double r, double theta, double z) : r_(r), theta_(theta), z_(z){};
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
    }

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
        void match_impl(); /// Actual implementation of the match code.

        STSServoDriver *servo;

        Action *chooseNextAction(
            std::vector<Action> &actions,
            RobotPosition currentPosition,
            MotionPlanner &motionPlanner);

        /// @brief Sets the left arm to a specific position
        /// @param armPosition the arm position
        bool set_left_arm_position(ArmPosition const& armPosition);

        /// @brief Sets the right arm to a specific position
        /// @param armPosition the arm position
        bool set_right_arm_position(ArmPosition const& armPosition);

        /// @brief Try to move an arm to a set position, returns false if could not be computed.
        bool set_arm_position(ArmPosition const& armPosition, unsigned char servoID);

        /// @brief  \brief Blocks until arms have finished moving
        void waitForArmMotion();

        /// @brief Execute the cake building sequence
        void build_cakes();
    };
}

#endif
