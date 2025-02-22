/// A simple test for the AS5045 encoder

#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include <miam_utils/drivers/NautilusWrapper.h>
#include <miam_utils/drivers/NautilusWrapper.h>
#include <miam_utils/drivers/STSServoDriver.h>
#include <miam_utils/drivers/PCA9635Driver.h>
#include <miam_utils/drivers/PCA9546ADriver.h>


#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    NautilusWrapper rightMotor(RPI_SPI_00, 6.0, 1000000), leftMotor(RPI_SPI_01, 6.0, 1000000);

    std::cout << "Enumerating components: " << std::endl;

    bool encoderState;
    std::cout << "Motor: ";
    std::cout << "right: " << (rightMotor.init(encoderState) ? "OK": "NOK");
    std::cout << "\tleft: " << (leftMotor.init(encoderState) ? "OK": "NOK");
    std::cout << std::endl;

    STSServoDriver servoDriver;
    bool servoPresent = servoDriver.init("/dev/serial0", -1);
    std::cout << "Serovs: " << (servoPresent ? "OK": "NOK") << std::endl;
    if (servoPresent)
    {
        std::cout << "Servo list: ";
        for (auto const& id : servoDriver.detectServos())
            std::cout << static_cast<int>(id) << "   ";
        std::cout << std::endl;
    }

    PCA9635 powerDriver;
    bool powerDriverPresent = powerDriver.init(&RPI_I2C, 0x6C, true);
    std::cout << "PCA9635: " << (powerDriverPresent ? "OK": "NOK") << std::endl;

    PCA9546A i2cExpander;
    std::cout << "PCA9546A: " << (i2cExpander.init(&RPI_I2C) ? "OK": "NOK") << std::endl;

    std::cout << std::endl << "Starting individual test: " << std::endl;
    if (powerDriverPresent)
    {
        std::cout << std::endl;
        for (int i = 0; i < 5; i++)
        {
            std::cout << "\rPCA9546A output 0: duty cycle 0.1" << std::flush;
            powerDriver.setDutyCycle(0, 0.1);
            usleep(1000000);
            std::cout << "\rPCA9546A output 0: duty cycle 0.0" << std::flush;
            powerDriver.setDutyCycle(0, 0.0);
            usleep(1000000);
        }
    }


    return 0;
}


