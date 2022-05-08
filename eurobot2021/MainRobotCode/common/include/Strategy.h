/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef STRATEGY_H
    #define STRATEGY_H

    #include "RobotInterface.h"
    #include "ServoHandler.h"

    class Strategy
    {
        public:
            // Constructor
            Strategy();

            // Called before the start of the match, to setup the robot.
            void setup(RobotInterface *robot, ServoHandler *servos);

            // The actual match code, which runs in its own thread.
            void match();

        private:
            void match_impl(); /// Actual implementation of the match code.
            RobotInterface *robot;
            ServoHandler *servo;

            // Actions
            bool handleStatue();
            bool moveSideSample();
            bool handleSideTripleSamples();
            bool moveThreeSamples();
            bool handleDigZone();
            bool pushSamplesBelowShelter();

            bool goBackToDigSite();

            // Utility functions
            void pushExcavationSite();
            bool shouldPushExcavationSite(ExcavationSquareColor color);
            void dropElements();
            ExcavationSquareColor testExcavationSite();
            void stopEverything();
    };

 #endif
