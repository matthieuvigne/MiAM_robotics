// A utility program to test the servos on the robot.
// To compile:
// arm-linux-gnueabihf-g++ TestServo.cpp -o TestServo `pkg-config --cflags --libs miam_utils_arm`

#include <miam_utils/drivers/MaestroServoDriver.h>

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


int main(int argc, char **argv)
{
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

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

        std::string str;
        std::cout << "Enter servo number: ";
        std::getline(std::cin, str);
        try
        {
            int servoNumber = std::stoi(str);
            if (servoNumber < 0 || servoNumber > 17)
            {
                throw std::invalid_argument("");
            }
            while (true)
            {
                std::cout << "Enter desired position (-1 to change servo): ";
                std::getline(std::cin, str);
                try
                {
                    int position = std::stoi(str);
                    if (position < 0)
                        break;
                    maestro.setPosition(servoNumber, position);
                }
                catch (const std::invalid_argument &)
                {
                    std::cout << "Invalid position: disabling servo" << std::endl;
                    maestro.setPosition(servoNumber, 0);
                }
            }

        }
        catch (const std::invalid_argument &)
        {
                std::cout << "Error: invalid servo number" << std::endl;
        }
    }
    return 0;
}

