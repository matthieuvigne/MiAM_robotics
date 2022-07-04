/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef STRATEGY_H
    #define STRATEGY_H

    #include "RobotInterface.h"
    #include "ServoHandler.h"

    //~ #include "CameraClient.h"
    #include <network/camera_client.hpp>

    class Strategy
    {
        public:
            // Constructor
            Strategy();

            // Called before the start of the match, to setup the robot.
            void setup(RobotInterface *robot);

            // The actual match code, which runs in its own thread.
            void match();


            network::CameraClient camera_;

            std::vector<pthread_t> createdThreads_;
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

            bool handleDigZone();
            bool is_handle_dig_zone_finished;
            bool is_bonus_already_counted;

            bool handleSideTripleSamples();
            bool is_handle_side_triple_samples_finished;

            bool goBackToDigSite();

            // Utility functions
            void pushExcavationSite();
            bool shouldPushExcavationSite(ExcavationSquareColor color);
            void dropElements();
            ExcavationSquareColor testExcavationSite();
            void stopEverything();
    };

 #endif
