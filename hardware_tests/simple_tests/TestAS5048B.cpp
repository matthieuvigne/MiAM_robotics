#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/AS5048BDriver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    AS5048B encoder;

    if (!encoder.init(&RPI_I2C))
    {
        std::cout << "Failed to init AS5048B" << std::endl;
        return -1;
    }

    while (true)
    {
        int const pos = encoder.getPosition();
        std::cout << "Position: " << pos << std::endl;
        usleep(10000);
    }

    return 0;
}


