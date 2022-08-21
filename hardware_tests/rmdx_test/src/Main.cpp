/// A simple test for the RMDX motors

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/MCP2515Driver.h>
#include <miam_utils/drivers/RMDX.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    SPIWrapper spi(RPI_SPI_00, 400000);
    MCP2515 mcp(&spi);
    if (!mcp.init())
    {
        std::cout << "Failed to talk to MCP" << std::endl;
        exit(0);
    }

    RMDX motor(&mcp, 0.1);

    int const motorRightId = 1;
    int const motorLeftId = 2;

    motor.enable(motorRightId);
    // FIXME: mcp needs to be re-init between every message.
    // mcp.init();
    // motor.enable(motorLeftId);
    usleep(4000000);

    // mcp.init();
    // motor.enable(motorRightId);
    motor.setSpeed(motorRightId, -5);
    // motor.setSpeed(motorLeftId, -5);
    // motor.setSpeed(motorRightId, -5);
    // motor.setSpeed(motorRightId, -5);
    // motor.setSpeed(motorRightId, -5);
    // motor.setSpeed(motorRightId, -5);
    usleep(1000000);
    motor.setSpeed(motorRightId, 5);
    usleep(1000000);
    // motor.setSpeed(motorLeftId, 5);

    // // mcp.init();
    // motor.setSpeed(motorRightId, 15);
    // usleep(5000);

    // // mcp.init();
    // motor.setSpeed(motorLeftId, 3);
    // usleep(1000000);
    // motor.stop(motorRightId);
    // motor.stop(motorLeftId);

    struct timespec startTime, currentTime;
    clock_gettime(CLOCK_MONOTONIC, &startTime);
    // mcp.init();
    motor.setSpeed(motorRightId, -1);
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    double const elapsed = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1e9;
    std::cout << elapsed << std::endl;

    return 0;
}


