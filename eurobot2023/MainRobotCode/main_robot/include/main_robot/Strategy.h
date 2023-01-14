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
        class Strategy: public AbstractStrategy
        {
            public:
                // Constructor
                Strategy();

                // Called before the start of the match, to setup the robot.
                void setup(RobotInterface *robot) override;

                // The actual match code, which runs in its own thread.
                void match() override;
            private:
                void match_impl(); /// Actual implementation of the match code.
                RobotInterface *robot;
                MotionController *motionController;
                ServoHandler *servo;

                // Actions
                bool handleStatue();
                bool is_handle_statue_finished;

                bool moveSideSample();
                bool is_move_side_sample_finished;

                bool moveThreeSamples();
                bool moveThreeSamplesBackup();
                bool is_move_three_samples_finished;

                bool pushSamplesBelowShelter();
                bool is_push_samples_below_shelter_finished;

                bool is_handle_dig_zone_finished;
                bool is_bonus_already_counted;

                bool handleSideTripleSamples();
                bool is_handle_side_triple_samples_finished;

                bool goBackToDigSite();

                // Utility functions
                void pushExcavationSite();
                void dropElements();
                void stopEverything();

                Action* chooseNextAction(
                    std::vector<Action>& actions,
                    RobotPosition currentPosition,
                    MotionPlanning motionPlanner
                );
        };
    }

 #endif
