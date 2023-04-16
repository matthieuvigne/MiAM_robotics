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

        enum ReservoirTilt
        {
            UP, DOWN
        };

        enum BrushDirection
        {
            OFF, TOWARDS_FRONT, TOWARDS_BACK
        };

        enum RailHeight // from 0 to 1000
        {
            BOTTOM = 0,
            TOP = 1000,
            CHERRY_DISTRIBUTOR = 200,
            CHERRY_BASKET = 800,
            MIDDLE = 500
        };

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

                STSServoDriver *servo;

                Action* chooseNextAction(
                    std::vector<Action>& actions,
                    RobotPosition currentPosition,
                    MotionPlanning motionPlanner
                );

                void move_rail(RailHeight railHeight);
                void set_brush_move(BrushDirection brushDirection);
                void set_reservoir_tilt(ReservoirTilt reservoirTilt);

                void grab_cherries();
                void put_cherries_in_the_basket();
        };
    }

 #endif
