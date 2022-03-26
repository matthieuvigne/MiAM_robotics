/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef STRATEGY_H
    #define STRATEGY_H

    #include "RobotInterface.h"
    #include "ServoHandler.h"
    // Function call before the start of the match, to setup the state of the robot (mostly, the servos).
    void setupRobot(RobotInterface *robot, ServoHandler *servos);

    // Main strategy function.
    void matchStrategy(RobotInterface *robot, ServoHandler *servos);
 #endif
