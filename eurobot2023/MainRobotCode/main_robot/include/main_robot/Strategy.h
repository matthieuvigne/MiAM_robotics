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
    #include "common/MotionPlanning.h"

    namespace main_robot{

        class ArmPosition 
        {
            public:
                double x_;
                double y_;
                double z_;
            
                ArmPosition(double x, double y, double z): x_(x), y_(y), z_(z) {};
        };

        class Strategy: public AbstractStrategy
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
            private:
                void match_impl(); /// Actual implementation of the match code.
 
                STSServoDriver *servo;


                Action* chooseNextAction(
                    std::vector<Action>& actions,
                    RobotPosition currentPosition,
                    MotionPlanning motionPlanner
                );

                void set_left_arm_position(ArmPosition armPosition);
                void set_right_arm_position(ArmPosition armPosition);
                void build_cakes();
    };
}

 #endif
