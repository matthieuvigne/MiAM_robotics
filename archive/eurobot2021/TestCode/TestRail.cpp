// A utility program to test rail motion on the main robot.
// To compile:
// arm-linux-gnueabihf-g++ TestRail.cpp ../MainRobotCode/embedded/src/uCListener.cpp -o TestRail `pkg-config --cflags --libs miam_utils_arm` -I../MainRobotCode/embedded/include/ -I../MainRobotCode/common/include/ -lpthread
#include <miam_utils/drivers/MaestroServoDriver.h>
#include "uCListener.h"

#include <stdexcept>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string>
MaestroDriver maestro;

void killCode(int x)
{
	for(int i = 0; i < 18; i++)
	{
	    maestro.setPosition(i, 0);
		usleep(80);
	}
    exit(0);
}


int MIAM_POTENTIOMETER_LOW_VALUE = 60;
int MIAM_POTENTIOMETER_HIGH_VALUE = 330;
const int MIAM_RAIL_TOLERANCE = 10;

const int MIAM_RAIL_SERVO_ZERO_VELOCITY = 1450;
const int MIAM_RAIL_SERVO_MAX_UP_VELOCITY = 2000;
const int MIAM_RAIL_SERVO_MAX_DOWN_VELOCITY = 1000;

// Controller parameters
namespace controller
{
    double const railKp = 20.0;
    double const railKd = 0.0;
    double const railKi = 0.0;
}
miam::PID PIDRail_ = miam::PID(controller::railKp, controller::railKd, controller::railKi, 0.1);

void moveRail(double const& position)
{
    // Compute target potentiometer value.
    int targetValue = MIAM_POTENTIOMETER_LOW_VALUE - (MIAM_POTENTIOMETER_LOW_VALUE - MIAM_POTENTIOMETER_HIGH_VALUE) * position;

    // Compute error
    int error = uCListener_getData().potentiometerPosition - targetValue;
    int nIter = 0;
    while (std::abs(error) > 8 && nIter < 120)
    {
        int targetVelocity = -PIDRail_.computeValue(error, 0.020);
        targetVelocity = std::max(
            std::min(
                MIAM_RAIL_SERVO_ZERO_VELOCITY + targetVelocity,
                MIAM_RAIL_SERVO_MAX_UP_VELOCITY
                ),
            MIAM_RAIL_SERVO_MAX_DOWN_VELOCITY
        );
        // Send target to servo
        maestro.setPosition(ELEVATOR ,targetVelocity);

        usleep(20000);
        error = uCListener_getData().potentiometerPosition - targetValue;
        std::cout << "Current: " << uCListener_getData().potentiometerPosition << " Target: " << targetValue << std::endl;
        nIter++;
    }
    maestro.setPosition(ELEVATOR, MIAM_RAIL_SERVO_ZERO_VELOCITY);
}


int main(int argc, char **argv)
{
    signal(SIGINT, killCode);

    bool failed = argc < 3;
    if (!failed)
    {
        try
        {
            MIAM_POTENTIOMETER_LOW_VALUE = std::stoi(argv[1]);
            MIAM_POTENTIOMETER_HIGH_VALUE = std::stoi(argv[2]);
        }
        catch (const std::invalid_argument &)
        {
            failed = true;
        }
    }
    if (!failed)
    {
        failed = MIAM_POTENTIOMETER_LOW_VALUE < 0 || MIAM_POTENTIOMETER_HIGH_VALUE > 1023;
        failed |= MIAM_POTENTIOMETER_LOW_VALUE >= MIAM_POTENTIOMETER_HIGH_VALUE;
    }
    if (failed)
    {
        std::cout << "Usage: ./TestRail <min_value> <max_value>" << std::endl;
        std::cout << "with 0 < min_value < max_value < 1023" << std::endl;
        exit(0);
    }

    if(!uCListener_start("/dev/arduinoUno"))
    {
        std::cout << "Failed to talk to arduino, exiting." << std::endl;
        exit(0);
    }

    if (!maestro.init("/dev/ttyACM2"))
    {
        std::cout << "Failed to talk to maestro, exiting." << std::endl;
        exit(0);
    }

	for(int i = 0; i < 18; i++)
	{
		maestro.setPosition(i, 0);
		usleep(80);
	}

    while (true)
    {
        std::cout << "Enter target position: ";
        std::string str;
        std::getline(std::cin, str);
        try
        {
            double pos = std::stof(str);
            if (pos < 0 || pos > 1)
            {
                throw std::invalid_argument("");
            }
            moveRail(pos);
        }
        catch (const std::invalid_argument &)
        {
                std::cout << "Error: invalid position" << std::endl;
        }
    }
    return 0;
}

