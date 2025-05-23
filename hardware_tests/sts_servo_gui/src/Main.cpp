/// A GUI to test  and configure the STS3215 servos of the robot

#include <gtkmm/application.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/STSServoDriver.h>

#include <cstdlib>
#include <iostream>

#include "MainWindow.h"

int main (int argc, char *argv[])
{
    // Try to communicate with servo.
    std::string portName = "/dev/ttyAMA0";
    if (argc > 1)
        portName = argv[1];

    STSServoDriver driver;

    if (!driver.init(portName, -1))
    {
        std::cout << "Failed to init communication with servos on " << portName << std::endl;
        return 0;
    }
    // Start all servos in position mode.
    driver.setMode(0xFE, STS::Mode::POSITION);
    usleep(10000);

    // Create GUI
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    MainWindow window(&driver);
    app->run(window);
    // Stop all motors
    driver.disable(0xFE);
    return 0;
}


