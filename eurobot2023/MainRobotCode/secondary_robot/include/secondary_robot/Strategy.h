/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SECONDARY_ROBOT_STRATEGY_H
    #define SECONDARY_ROBOT_STRATEGY_H

    #include "common/RobotInterface.h"
    #include "common/ServoHandler.h"
    #include "common/AbstractAction.h"
    #include "common/AbstractStrategy.h"
    #include "common/MotionPlanning.h"

    namespace secondary_robot{
        class Strategy: public AbstractStrategy
        {
            public:
                // Constructor
                Strategy();

                // Called before the start of the match, to setup the robot.
                void setup(RobotInterface *robot);

                // Code executed when shutting down the robot
                void shutdown() override;

                // The actual match code, which runs in its own thread.
                void match();
            private:
                void match_impl(); /// Actual implementation of the match code.
                RobotInterface *robot;
                MotionController *motionController;
                STSServoDriver *servo;

                Action* chooseNextAction(
                    std::vector<Action>& actions,
                    RobotPosition currentPosition,
                    MotionPlanning motionPlanner
                );
        };
    }

 #endif
