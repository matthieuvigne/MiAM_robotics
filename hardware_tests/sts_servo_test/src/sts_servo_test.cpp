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

    std::string userInput;
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
    ServoManager servo_manager;
    ServoManager *servoManager_ = &servo_manager;
    Robot *robot_ = &robot;
    servo_manager.init(&robot);

    while (!servo_manager.isRailCalibDone())
        robot_->wait(0.1);
    std::cout << "Calib done" << std::endl;

    /////////////// Banner test

    // servo_manager.backRail_.move(0.3);
    // while (servo_manager.backRail_.isMoving())
    //     robot_->wait(0.1);
    // servo_manager.foldBackPlank();
    // std::cout << "Calib done" << std::endl;

    //   std::cout << "Press enter to continue" << std::endl;
    //   std::cin >> userInput;
    // while (true)
    // {
    //   servo_manager.dropBanner();
    //   robot.wait(1.5);
    //   servo_manager.backClawOpen();
    //   std::cout << "Press enter to continue" << std::endl;
    //   std::cin >> userInput;
    //   robot.getServos()->setTargetPosition(31, 1750);
    //   servo_manager.foldClaws(true);
    //   std::cout << "Press enter to continue" << std::endl;
    //   std::cin >> userInput;

    // }

    /////////////////////////////////////////

    // servo_manager.grabPlank();
    // servo_manager.releasePlank();
    // servo_manager.backRail_.move(0.05);
    // servo_manager.frontRightClaw_.rail_.move(0.05);
    // servo_manager.frontLeftClaw_.rail_.move(0.05);
    // std::cout << "Press enter to continue" << std::endl;
    // std::cin >> userInput;

    // while (true)
    // {
    //     servo_manager.backClawClose();
    //     servo_manager.frontClawClose();
    //     servo_manager.frontRightClaw_.closeClaw();
    //     servo_manager.frontLeftClaw_.closeClaw();
    //     robot_->wait(0.3);

    //     std::cout << "Front:" << std::endl;
    //     std::cout << servo_manager.checkGrab(true) << std::endl;
    //     std::cout << servo_manager.areBothFrontSideClawsFull() << std::endl;
    //     std::cout << "Back:" << std::endl;
    //     std::cout << servo_manager.checkGrab(false) << std::endl;

    //     std::cout << "Press enter to continue" << std::endl;
    //     std::cin >> userInput;


    //     std::cout << "Front:" << std::endl;
    //     std::cout << servo_manager.checkGrab(true) << std::endl;
    //     std::cout << "Back:" << std::endl;
    //     std::cout << servo_manager.checkGrab(false) << std::endl;

    //     servo_manager.backClawOpen();
    //     servo_manager.frontClawOpen();
    //     servo_manager.frontRightClaw_.openClaw();
    //     servo_manager.frontLeftClaw_.openClaw();
    //     robot_->wait(0.5);
    //     std::cout << "Press enter to continue" << std::endl;
    //     std::cin >> userInput;


    //     std::cout << "Front:" << std::endl;
    //     std::cout << servo_manager.checkGrab(true) << std::endl;
    //     std::cout << "Back:" << std::endl;
    //     std::cout << servo_manager.checkGrab(false) << std::endl;

    // }
    ///////////////////////////////////////
    // Back test
    ///////////////////////////////////////

    // bool success = false;
    // while (true) {

    //   // Prepare the grab
    //   servo_manager.releaseBackPlank();
    //   servo_manager.prepareGrab(false);
    //   while (servo_manager.railManager_.areAnyMoving())
    //       robot_->wait(0.050);

    //   // Try grab the columns
    //   std::cout << "Press enter to try grab" << std::endl;
    //   std::cin >> userInput;
    //   servo_manager.releaseBackPlank();
    //   robot_->wait(0.1);
    //   success = servo_manager.grab(false);
    //   robot_->wait(0.1);
    //   if(success)
    //   {
    //     std::cout << "GRAB SUCCESS!" << std::endl;
    //   } else {
    //     std::cout << "GRAB FAILURE!" << std::endl;
    //     std::cout << "Press enter to retry" << std::endl;
    //     std::cin >> userInput;
    //     servo_manager.backClawOpen();
    //     //robot_->getMotionController()->goStraight(50, 0.5);
    //     //robot_->getMotionController()->waitForTrajectoryFinished();
    //   }
    // }

    // // std::cout << "Press enter to continue" << std::endl;
    // // std::cin >> userInput;
    // // servo_manager.grabBackTwoPlanks();
    // // servo_manager.backClawClose();

    // // std::cout << "Press enter to continue" << std::endl;
    // // std::cin >> userInput;
    // // servo_manager.backRail_.move(1.0);

    // // while (true) ;;
    ///////////////////////////////////////


    ///////////////////////////////////////
    // Front test
    ///////////////////////////////////////
    while (true)
    {
        servo_manager.prepareGrab(true);
        while (servo_manager.railManager_.areAnyMoving())
            robot_->wait(0.050);
        servo_manager.frontClawOpen();

        std::cout << "hello" << std::endl;
        std::cout << "Press enter to continue" << std::endl;
        std::cin >> userInput;
        std::cout << "world" << std::endl;
        bool success = servo_manager.grab(true);


        servo_manager.checkGrab(true);
        servo_manager.areBothFrontSideClawsFull();


        std::cout << "Press enter to continue" << std::endl;
        std::cin >> userInput;


        std::cout << "Second time check grab" << std::endl;

        servo_manager.checkGrab(true);
        servo_manager.areBothFrontSideClawsFull();

        std::cout << "Press enter to continue" << std::endl;
        std::cin >> userInput;


        servo_manager.buildFrontTower();
    }


    ///////////////////////////////////////


    // robot.getServos()->setMode(0xFE, STS::Mode::POSITION);

    // Claw leftClaw(robot.getServos(), RailServo(robot.getServos(), 10, 24, 9500, false), 12, 11, 825, true);
    // Claw rightClaw(robot.getServos(), RailServo(robot.getServos(), 13, 23, 9500, true), 14, 15, 575, false);

    // MiddleClaw middleClaw(robot.getServos(), RailServo(robot.getServos(), 6, 21, 6000, false));

    // middleClaw.rail_.startCalibration();

    // rightClaw.move(ClawPosition::FORWARD);
    // rightClaw.rail_.startCalibration();

    // leftClaw.move(ClawPosition::FORWARD);
    // leftClaw.rail_.startCalibration();

    // middleClaw.open();
    // middleClaw.rail_.startCalibration();

    // while (!rightClaw.rail_.isCalibrated() || !leftClaw.rail_.isCalibrated() || !middleClaw.rail_.isCalibrated())
    //     usleep(50000);

    // while (true)
    // {
    //     rightClaw.rail_.move(0.0);
    //     leftClaw.rail_.move(0.0);
    //     middleClaw.rail_.move(0.0);
    //     while (rightClaw.rail_.isMoving() || leftClaw.rail_.isMoving() || middleClaw.rail_.isMoving())
    //         usleep(50000);
    //     rightClaw.move(ClawPosition::FORWARD);
    //     rightClaw.openClaw();
    //     leftClaw.move(ClawPosition::FORWARD);
    //     leftClaw.openClaw();
    //     middleClaw.open();

    //     usleep(3000000);

    //     rightClaw.closeClaw();
    //     leftClaw.closeClaw();
    //     middleClaw.close();
    //     usleep(250000);
    //     // rightClaw.move(ClawPosition::SIDE);
    //     rightClaw.rail_.move(0.7);

    //     // leftClaw.move(ClawPosition::SIDE);
    //     leftClaw.rail_.move(0.7);
    //     middleClaw.rail_.move(0.9);

    //     while (rightClaw.rail_.isMoving() || leftClaw.rail_.isMoving() || middleClaw.rail_.isMoving())
    //         usleep(50000);

    //     usleep(3000000);

    //     rightClaw.openClaw();
    //     leftClaw.openClaw();
    // }


    // RailServo middleRail(20, 20, 5500, true);
    // // RailServo middleRail(10, 24, 9500, false);
    // middleRail.init(robot.getServos());

    // std::vector<RailServo*> rails;
    // rails.push_back(&middleRail);

    // RailManager railManager;

    // railManager.start(rails);

    // while (!railManager.areCalibrated())
    //     usleep(50000);

    // while (true)
    // {
    //     middleRail.move(0.05);
    //     while (middleRail.isMoving())
    //         usleep(50000);
    //     std::cout << "Bottom" << std::endl;
    //     usleep(1000000);
    //     middleRail.move(1.0);
    //     while (middleRail.isMoving())
    //         usleep(50000);
    //     std::cout << "Top" << std::endl;
    //     usleep(1000000);
    // }

    // RailServo middleRail(robot.getServos(), 5, 22, 9500, true, true);

    // std::vector<RailServo*> rails;
    // rails.push_back(&middleRail);

    // RailManager railManager;

    // railManager.start(rails);

    // while (!railManager.areCalibrated())
    //     usleep(50000);

    // while (true)
    // {
    //     servos->setTargetPosition(6, 500);
    //     servos->setTargetPosition(7, 500);
    //     middleRail.move(0.0);
    //     while (middleRail.isMoving())
    //         usleep(50000);
    //     usleep(1000000);

    //     servos->setTargetPosition(6, 500);
    //     servos->setTargetPosition(7, 500);
    //     usleep(2000000);


    //     middleRail.move(0.5);
    //     while (middleRail.isMoving())
    //         usleep(50000);
    // }

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
