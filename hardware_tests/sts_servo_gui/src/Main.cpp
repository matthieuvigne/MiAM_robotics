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

    RPi_enablePorts();

    STSServoDriver driver;

    if (!driver.init("/dev/ttyAMA0", -1))
    {
        std::cout << "Failed to init communication with servos." << std::endl;
        return 0;
    }
    // Start all servos in position mode.
    driver.setMode(0xFE, STS::Mode::POSITION);
    driver.setTorqueLimit(0x01, 0.1);
    usleep(10000);

    // Create GUI
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    MainWindow window(&driver);
    app->run(window);
    // Stop all motors
    driver.disable(0xFE);
    return 0;
}


