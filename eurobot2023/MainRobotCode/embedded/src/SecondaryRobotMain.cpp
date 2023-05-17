/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#include "Robot.h"
#include "common/RobotGUI.h"
#include "secondary_robot/Parameters.h"
#include "secondary_robot/Strategy.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <filesystem>
#include <fstream>

Robot *robotPtr;

std::string const lockFile = "/tmp/miam_lock";

bool checkLockFile()
{
    if (std::filesystem::exists(lockFile))
        return false;
    std::ofstream file(lockFile);
    return true;
}

// Stop motor before exit.
void killCode(int x)
{
    std::filesystem::remove(lockFile);
    robotPtr->shutdown();
    exit(0);
}



int main(int argc, char **argv)
{
    setpriority(PRIO_PROCESS, 0, -20);

    if (!checkLockFile())
    {
        std::cout << "Could not start: lock file " << lockFile << " already exists. Is another code running?" << std::endl;
        return -1;
    }
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
            std::cout << "Secondary robot code." << std::endl;
            std::cout << "Usage: ./secondaryRobotCode [--testmode] [--nolidar]." << std::endl;
            exit(0);
        }
    }
    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();

    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Create objects
    Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();
    RobotGUI gui;

    secondary_robot::Strategy strategy;
    Robot robot(secondary_robot::generateParams(), &strategy, &gui, testMode, noLidar);
    robotPtr = &robot;

    // Start low-level loop
    std::thread loop(&Robot::lowLevelLoop, &robot);

    // Start gui
    gui.show_all();
    app->run(gui);
    return 0;
}

