#include "Robot.h"
#include "common/RobotGUI.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"
#include "main_robot/ServoManager.h"

#include "main_robot/RailServo.h"

#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <filesystem>
#include <fstream>

STSServoDriver *servos;

void kill(int)
{
    servos->disable(0xFE);
    exit(0);
}

int main(int argc, char* argv[])
{
    // Try to communicate with servo.
    RPi_enablePorts();

    // Create objects
    Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();
    RobotGUI gui;
    bool testMode = true;
    bool noLidar = true;
    main_robot::Strategy strategy;
    Robot robot(main_robot::generateParams(), &strategy, &gui, testMode, noLidar, false);

    servos = robot.getServos();
    if (!servos->init("/dev/ttyAMA0", -1))
    {
        std::cout << "Failed to init communication with servos." << std::endl;
        return 0;
    }
    signal(SIGINT, kill);
    signal(SIGTERM, kill);

    // Instantiate the servo manager
    // ServoManager servo_manager;
    // ServoManager *servoManager_ = &servo_manager;
    // Robot *robot_ = &robot;
    // servo_manager.init(&robot, false);
    // robot.wait(1.0);

    RailServo rail(robot.getServos(), 10, 24, 9500, false);

    rail.startCalibration();

    while (!rail.isCalibrated())
        usleep(100000);

    while (true)
    {

        rail.move(0.5);
        usleep(100000);

        while (rail.isMoving())
            usleep(20000);

        std::cout << "Target reached" << std::endl;

        usleep(1000000);

        rail.move(0.0);
        usleep(100000);

        while (rail.isMoving())
            usleep(20000);

        std::cout << "Target reached" << std::endl;

        usleep(1000000);
    }

    return EXIT_SUCCESS;
}
