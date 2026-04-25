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
    Robot robot(main_robot::generateParams(), &strategy, &gui, testMode, noLidar, false, false);

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

    std::string input;

    // while (true)
    // {
    //     servoManager_->moveArm(ArmPosition::GRAB);
    //     std::getline(std::cin, input);
    //     servoManager_->moveArm(ArmPosition::RAISE);
    //     std::getline(std::cin, input);

    // }

    // while (true)
    // {
    //     servo_manager.pumpOn(Side::RIGHT);
    //     std::cout << "Right pump on" << std::endl;
    //     std::getline(std::cin, input);

    //     servo_manager.pumpOff(Side::RIGHT);
    //     std::cout << "Right pump off" << std::endl;
    //     std::getline(std::cin, input);

    //     servo_manager.pumpOn(Side::LEFT);
    //     std::cout << "Left pump on" << std::endl;
    //     std::getline(std::cin, input);

    //     servo_manager.pumpOff(Side::LEFT);
    //     std::cout << "Left pump off" << std::endl;
    //     std::getline(std::cin, input);
    // }

    // while (!servos->areAllRailsCalibrated())
    //     robot_->wait(0.1);
    // std::cout << "Calib done" << std::endl;
    // servoManager_->moveRails(RailPosition::FORWARD);
    // while (servoManager_->areRailsMoving())
    //     robot_->wait(0.1);
    // std::cout << "Init done" << std::endl;


    // servoManager_->moveArm(ArmPosition::RAISE);

    // while (true)
    // {
    //     servoManager_->translateSuction(Side::LEFT, 0.0);
    //     std::cout << "Left 0" << std::endl;
    //     std::getline(std::cin, input);
    //     servoManager_->translateSuction(Side::LEFT, 1.0);
    //     std::cout << "Left 1" << std::endl;
    //     std::getline(std::cin, input);
    //     servoManager_->translateSuction(Side::RIGHT, 0.0);
    //     std::getline(std::cin, input);
    //     servoManager_->translateSuction(Side::RIGHT, 1.0);
    //     std::getline(std::cin, input);
    // }

    // while (true)
    // {
    //     servoManager_->pumpOn(Side::LEFT);
    //     servoManager_->pumpOn(Side::RIGHT);
    //     std::getline(std::cin, input);
    //     servoManager_->pumpOff(Side::LEFT);
    //     servoManager_->pumpOff(Side::RIGHT);
    //     std::getline(std::cin, input);
    // }
    while (true)
    {
        std::getline(std::cin, input);
        std::cout << "hiding arm" << std::endl;
        servoManager_->hideArm();
        robot_->wait(1.0);
        std::getline(std::cin, input);
        std::cout << "grabbing crates" << std::endl;
        servoManager_->grabCrates();
    }

    // std::string input;
    // while (true)
    // {
    //     servoManager_->moveArm(ArmPosition::GRAB);
    //     servoManager_->pumpOn(Side::RIGHT);
    //     servoManager_->pumpOn(Side::LEFT);
    //     std::getline(std::cin, input);
    //     servoManager_->moveArm(ArmPosition::RAISE);
    //     std::getline(std::cin, input);
    //     servoManager_->moveArm(ArmPosition::FOLD_MID);
    //     robot_->wait(0.5);
    //     servoManager_->moveArm(ArmPosition::FOLD);
    //     robot_->wait(0.5);
    //     servoManager_->moveRails(RailPosition::INTERNAL);
    //     while (servoManager_->areRailsMoving())
    //         robot_->wait(0.1);
    //     servoManager_->pumpOff(Side::RIGHT);
    //     servoManager_->pumpOff(Side::LEFT);

    //     std::getline(std::cin, input);

    //     std::vector<Tag> tags = visionHandler.getTags();
    //     std::cout << "Tags: ";
    //     for (auto const& t : tags)
    //     {
    //         if (t.markerId == 36)
    //             std::cout << "blue ";
    //         if (t.markerId == 47)
    //             std::cout << "yellow ";
    //     }
    //     std::cout << std::endl;
    //     std::getline(std::cin, input);

    //     servoManager_->moveRails(RailPosition::FORWARD);
    //     while (servoManager_->areRailsMoving())
    //         robot_->wait(0.1);
    //     servoManager_->moveArm(ArmPosition::RAISE);
    //     std::getline(std::cin, input);

    //     // TODO: move
    //     if (tags.size() == 4)
    //     {

    //         double translation = tags[1].markerId == 36 ? 1.0 : 0.0;
    //         servoManager_->translateSuction(Side::LEFT, translation);
    //         translation = tags[3].markerId == 36 ? 1.0 : 0.0;
    //         servoManager_->translateSuction(Side::RIGHT, translation);
    //     }
    // }
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
