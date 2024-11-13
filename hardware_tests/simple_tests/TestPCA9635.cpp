/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/PCA9635Driver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    PCA9635 device;

    if (!device.init(&RPI_I2C, 0x40, true))
    {
        std::cout << "Failed to init PCA9635" << std::endl;
        return -1;
    }

    while (true)
    {
        device.setDutyCycle(1, 0.2);
        std::cout << "Set DC: 0.5" << std::endl;
        usleep(1000000);
        device.setDutyCycle(1, 0.0);
        std::cout << "Set DC: 0.0" << std::endl;
        usleep(1000000);
    }

    return 0;

    // Res: YC248-JR-071KL ; YC248-JR-0747KL


}


