/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#include "Robot.h"
#include "common/RobotGUI.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>

Robot *robotPtr;


// Stop motor before exit.
void killCode(int x)
{
    robotPtr->shutdown();
    exit(0);
}


int main(int argc, char **argv)
{
    setpriority(PRIO_PROCESS, 0, -20);

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
    // Turn left pump off - it is on by default.
    RPi_setupGPIO(12, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(12, false);
    RPi_setupGPIO(13, PiGPIOMode::PI_GPIO_OUTPUT);
    RPi_writeGPIO(13, false);

    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Create objects
    Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();
    RobotGUI gui;

    main_robot::Strategy strategy;
    Robot robot(main_robot::generateParams(), &strategy, &gui, testMode, noLidar);
    robotPtr = &robot;

    // Start low-level loop
    std::thread loop(&Robot::lowLevelLoop, &robot);

    // Start gui
    gui.show_all();
    app->run(gui);
    return 0;
}

