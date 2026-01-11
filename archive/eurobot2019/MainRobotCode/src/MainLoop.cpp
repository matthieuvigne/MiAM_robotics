/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#include "Robot.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

Robot robot;


// Stop motor before exit.
void killCode(int x)
{
    robot.servos_.openTube(0);
    robot.servos_.openTube(1);
    robot.servos_.openTube(2);
    robot.servos_.tapOpen();
    robot.servos_.shutdownServos();
    robot.servos_.turnOffPump();
    robot.stopMotors();
    robot.lidar_.stop();
    exit(0);
}


int main(int argc, char **argv)
{
    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);
    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();

    // Start low-level loop.
    robot.lowLevelLoop();
    return 0;
}

