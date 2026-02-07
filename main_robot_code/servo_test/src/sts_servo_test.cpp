#include "Robot.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"

#include <iostream>
#include <signal.h>

STSScheduler *servos;

void kill(int)
{
    servos->shutdown();
    exit(0);
}

int main(int argc, char* argv[])
{
    std::string userInput;
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
    ServoManager servo_manager;
    ServoManager *servoManager_ = &servo_manager;
    Robot *robot_ = &robot;
    servo_manager.init(&robot);

    while (!servos->areAllRailsCalibrated())
        robot_->wait(0.1);
    std::cout << "Calib done" << std::endl;

    // while (true)
    // {
    //     servo_manager.prepareGrab(true);
    //     while (servo_manager.railManager_.areAnyMoving())
    //         robot_->wait(0.050);
    //     servo_manager.frontClawOpen();

    //     std::cout << "hello" << std::endl;
    //     std::cout << "Press enter to continue" << std::endl;
    //     std::cin >> userInput;
    //     std::cout << "world" << std::endl;
    //     bool success = servo_manager.grab(true);


    //     servo_manager.checkGrab(true);
    //     servo_manager.areBothFrontSideClawsFull();


    //     std::cout << "Press enter to continue" << std::endl;
    //     std::cin >> userInput;


    //     std::cout << "Second time check grab" << std::endl;

    //     servo_manager.checkGrab(true);
    //     servo_manager.areBothFrontSideClawsFull();

    //     std::cout << "Press enter to continue" << std::endl;
    //     std::cin >> userInput;


    //     servo_manager.buildFrontTower();
    // }



    return EXIT_SUCCESS;
}
