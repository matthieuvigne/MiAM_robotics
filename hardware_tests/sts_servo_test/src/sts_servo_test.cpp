#include "Robot.h"
#include "common/RobotGUI.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"
#include "main_robot/ServoManager.h"

#include "main_robot/RailServo.h"
#include "main_robot/Claw.h"

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

    robot.getServos()->setMode(0xFE, STS::Mode::POSITION);

    STSServoDriver *servos = robot.getServos();
    // Claw leftClaw(robot.getServos(), RailServo(robot.getServos(), 10, 24, 9500, false), 12, 11, 800, true);
    // Claw rightClaw(robot.getServos(), RailServo(robot.getServos(), 13, 23, 9500, true), 14, 15, 650, false);

    // rightClaw.rail_.startCalibration();
    // leftClaw.rail_.startCalibration();
    // while (!rightClaw.rail_.isCalibrated() || !leftClaw.rail_.isCalibrated())
    //     usleep(50000);

    // while (true)
    // {
    //     rightClaw.rail_.move(0.0);
    //     leftClaw.rail_.move(0.0);
    //     while (rightClaw.rail_.isMoving() || leftClaw.rail_.isMoving())
    //         usleep(50000);
    //     rightClaw.openClaw();
    //     leftClaw.openClaw();
    //     rightClaw.unfold();
    //     leftClaw.unfold();

    //     usleep(3000000);

    //     rightClaw.closeClaw();
    //     leftClaw.closeClaw();
    //     usleep(250000);
    //     rightClaw.rail_.move(0.7);
    //     leftClaw.rail_.move(0.7);
    //     while (rightClaw.rail_.isMoving() || leftClaw.rail_.isMoving())
    //         usleep(50000);
    //     // rightClaw.fold();
    //     // leftClaw.fold();

    //     usleep(3000000);
    // }


    RailServo middleRail(robot.getServos(), 5, 22, 9500, true, true);

    middleRail.startCalibration();
    while (!middleRail.isCalibrated())
        usleep(50000);

    while (true)
    {
        servos->setTargetPosition(6, 500);
        servos->setTargetPosition(7, 500);
        middleRail.move(0.0);
        while (middleRail.isMoving())
            usleep(50000);
        usleep(1000000);

        servos->setTargetPosition(6, 500);
        servos->setTargetPosition(7, 500);
        usleep(2000000);


        middleRail.move(0.5);
        while (middleRail.isMoving())
            usleep(50000);
    }

    // claw.rail_.startCalibration();
    // while (!claw.rail_.isCalibrated())
    //     usleep(50000);

    // claw.rail_.move(0.2);
    // while (claw.rail_.isMoving())
    //     usleep(50000);

    // while (true)
    // {
    //     claw.openClaw();
    //     claw.rail_.move(0.0);
    //     while (claw.rail_.isMoving())
    //         usleep(50000);
    //     claw.unfold();

    //     usleep(3000000);

    //     claw.closeClaw();
    //     usleep(100000);
    //     claw.rail_.move(0.7);
    //     while (claw.rail_.isMoving())
    //         usleep(50000);
    //     claw.fold();

    //     usleep(3000000);
    // }



    return EXIT_SUCCESS;
}
