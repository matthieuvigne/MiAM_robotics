/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/raspberry_pi/RaspberryPi.h"
#include <stdio.h>
#include <stdlib.h>

// See header directly for documentation of these constants.
const std::string RPI_SPI_00 = "/dev/spidev0.0";
const std::string RPI_SPI_01 = "/dev/spidev0.1";
I2CAdapter RPI_I2C;

void RPi_enablePorts()
{
    // Open file descriptors for I2C interfaces.
    bool i2cStarted = i2c_open(&RPI_I2C, "/dev/i2c-1");
    if(!i2cStarted)
    {
        printf("Could not open I2C port; perhaps I2C is not enabled? Exiting...\n");
        exit(-1);
    }
    if(!RPi_enableGPIO())
    {
        printf("Error accessing GPIO ports. Exiting...\n");
        exit(-1);
    }
}
