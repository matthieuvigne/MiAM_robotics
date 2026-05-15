#include "Robot.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"
#include "DemoGUI.h"

#include <iostream>
#include <signal.h>

STSScheduler *servos;

void kill(int)
{
    servos->shutdown();
    exit(0);
}

void wait_gui(DemoGUI *gui)
{
    while (!gui->wasClicked_)
        usleep(50000);
    gui->wasClicked_ = false;
}

void run_demo(DemoGUI *gui)
{
    gui->set_label("Initializing", "Please wait");

    RobotGUI rgui(true);
    bool testMode = true;
    bool noLidar = true;
    bool disableServos = false;
    main_robot::Strategy strategy;
    Robot robot(main_robot::generateParams(), &strategy, &rgui, testMode, noLidar, false, disableServos);

    servos = robot.getServos();
    while (!servos->init("/dev/ttyAMA0", -1))
    {
        gui->set_label("Failed to init servos", "");
        usleep(200000);
    }

    // Instantiate the servo manager
    ServoManager servo_manager;
    ServoManager *servoManager_ = &servo_manager;
    Robot *robot_ = &robot;
    signal(SIGINT, kill);
    signal(SIGTERM, kill);
    servo_manager.init(&robot);
    while (!servo_manager.getVisionHandler()->init())
    {
        gui->set_label("Camera init failed", "");
        usleep(200000);
    }
    while (!robot.getServos()->areAllRailsCalibrated())
        robot_->wait(0.1);
    servo_manager.moveRails(RailPosition::FORWARD);
    while (servoManager_->areRailsMoving())
        robot_->wait(0.1);
    ////////////////////////////////
    // Camera-based test
    while (true)
    {
        gui->set_label("Hiding arm", "");
        servoManager_->hideArm();
        robot_->wait(1.0);
        gui->set_label("Read to grab", "Grab !");
        bool wasGrabbed = false;
        while (!wasGrabbed)
        {
            wait_gui(gui);
            auto res = servoManager_->cameraDetectCrates();
            if (res.cratesPresent)
            {
                gui->set_label("Grabbing !", "");
                servoManager_->unhideArm();
                robot_->wait(0.4);
                servoManager_->grabCrates(res);
                wasGrabbed = true;
                gui->set_label("Grab done", "Drop");
                wait_gui(gui);
                servoManager_->pumpOff(Side::RIGHT);
                servoManager_->pumpOff(Side::LEFT);
                servoManager_->releaseSuction();
                gui->set_label("Grab done", "Empty robot");
                wait_gui(gui);
                servoManager_->emptyBed();
                gui->set_label("Grab done", "Hide arm");
                wait_gui(gui);
            }
            else
            {
                gui->set_label("Crates not detected, try again.", "Grab !");
                wasGrabbed = false;
            }
        }
        wait_gui(gui);
    }
}

int main(int argc, char* argv[])
{
    RPi_enablePorts();

    // Create objects
    Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();


    DemoGUI demoGUI;
    std::thread demo_th(run_demo, &demoGUI);
    demoGUI.show_all();
    app->run(demoGUI);
    return 0;
}
