/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#include "Robot.h"
#include "CameraClient.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

Robot *robotPtr;


// Stop motor before exit.
void killCode(int x)
{
    robotPtr->servos_.shutdownServos();
    robotPtr->servos_.activatePump(false);
    robotPtr->stopMotors();
    robotPtr->lidar_.stop();
    exit(0);
}


int main(int argc, char **argv)
{
    // Parse input
    bool testMode = false;
    bool noLidar = false;
    std::vector <std::string> sources;
    std::string destination;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--testmode")
            testMode = true;
        else if (std::string(argv[i]) == "--nolidar")
            noLidar = true;
        else
        {
            std::cout << "Main robot code." << std::endl;
            std::cout << "Usage: ./mainRobotCode [--testmode] [--nolidar]." << std::endl;
            exit(0);
        }
    }
    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();

    Robot robot(testMode, noLidar);
    robotPtr = &robot;

    // CameraClient client;
    // while(!client.init("192.168.6.42", 30000))
    //     usleep(500000);
    // client.run();

    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Start low-level loop.
    robot.lowLevelLoop();
    return 0;
}

