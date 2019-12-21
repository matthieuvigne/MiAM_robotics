/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <stdlib.h>
#include <stdio.h>
#include <fstream>

#include "MiAMEurobot/beaglebone/BeagleboneCape.h"
#include "MiAMEurobot/beaglebone/BBBGpio.h"

// See header directly for documentation of these constants.
const std::string SPI_0 = "/dev/spidev1.0";
const std::string SPI_10 = "/dev/spidev2.0";
const std::string SPI_11 = "/dev/spidev2.1";
I2CAdapter I2C_1;
const int CAPE_ANALOG[CAPE_N_ANALOG] = {0, 1, 2, 3, 4, 5, 6};
const int CAPE_DIGITAL[CAPE_N_DIGITAL] = {66, 67, 69, 68, 45, 44, 26};
const int CAPE_LED[CAPE_N_LED] = {47, 46};

// Internal function : access the cape manager to check if Eurobot cape is enabled. Exits if file access is not granted.
bool isEurobotEnabled()
{
    // Look in slots file if the Eurobot overlay is already enabled.
    std::ifstream file("/sys/devices/bone_capemgr.9/slots");
    if(!file.is_open())
    {
        printf("Enabling serial ports failed: cannot open cape manager (/sys/devices/bone_capemgr.9/slots).\n");
        exit(0);
    }

    bool isEurobotEnabled = false;
    // Check if the overlay is already enabled
    while (!file.eof())
    {
        std::string line;
        getline(file, line);
        if(line.find("ADAFRUIT-SPI1") != std::string::npos)
        {
            isEurobotEnabled = true;
            break;
        }
    }
    file.close();
    return isEurobotEnabled;
}


void BBB_enableCape()
{
    // Check if the overlay is already enabled.
    if(!isEurobotEnabled())
    {
        //~ std::string overlayFile = "/lib/firmware/Eurobot-00A0.dtbo";
        // Else, let us first check that the overlay file exists.
        //~ std::ifstream f(overlayFile);
        //~ if (!f.good())
        //~ {
            //~ printf("Enabling serial ports failed: cannot find overlay (%s).\n", overlayFile);
            //~ exit(-1);
        //~ }
        // Enable the overlay.
        system("echo BB-PWM2 > /sys/devices/bone_capemgr.9/slots");
        system("echo ADAFRUIT-SPI1 > /sys/devices/bone_capemgr.9/slots");
        system("echo PyBBIO-ADC > /sys/devices/bone_capemgr.9/slots");

        // Check that the overlay is indeed enabled.
        if(!isEurobotEnabled())
        {
            printf("Enabling serial ports failed: unknown error.\n");
            exit(-1);
        }
    }

    // Open file descriptors for I2C interfaces.
    bool i2cStarted = i2c_open(&I2C_1, "/dev/i2c-1");
    if(!i2cStarted)
    {
        printf("Could not open I2C port; perhaps overlay file is invalid ? Exiting...\n");
        exit(-1);
    }
    // Analog ports do not need to be enable.
    // Set all exposed GPIOs as input, except LEDs which should be outputs, set to low.
    for(int i = 0; i < CAPE_N_DIGITAL; i++)
        gpio_exportPin(CAPE_DIGITAL[i], "in");

    for(int i = 0; i < CAPE_N_LED; i++)
    {
        gpio_exportPin(CAPE_LED[i], "out");
        gpio_digitalWrite(CAPE_LED[i], 0);
    }
    // Enable PWM ports.
    system("echo 0 > /sys/class/pwm/export");
    system("echo 1 > /sys/class/pwm/export");
}
