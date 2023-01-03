    /// A simple test for the RMDX motors

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/MCP2515Driver.h>
#include <miam_utils/drivers/RMDX.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

struct timespec startTime;

void setStart()
{
    clock_gettime(CLOCK_MONOTONIC, &startTime);
}
double time()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    double const elapsed = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1e9;
}

int main (int argc, char *argv[])
{
    setStart();
    RPi_enablePorts();

    SPIWrapper spi(RPI_SPI_00, 500000);
    MCP2515 mcp(&spi);
    if (!mcp.init())
    {
        std::cout << "Failed to talk to MCP" << std::endl;
        exit(0);
    }

    RMDX motor(&mcp);

    int const motorRightId = 1;
    int const motorLeftId = 2;

    if (!motor.init(motorRightId))
    {
        std::cout << "Failed to init right motor, id " << motorRightId << std::endl;
        return -1;
    }
    if (!motor.init(motorLeftId))
    {
        std::cout << "Failed to init left motor, id " << motorLeftId << std::endl;
        return -1;
    }

    while (time() < 6)
    {
        motor.setCurrent(motorRightId, 0);
        motor.setCurrent(motorLeftId, 0);
    }

    while (time() < 6.8)
    {
        motor.setCurrent(motorRightId, -3);
        motor.setCurrent(motorLeftId, 3);
    }

    while (time() < 20.0)
    {
        motor.setCurrent(motorRightId, 0);
        motor.setCurrent(motorLeftId, 0);
    }
    // while (time() < 15)
    // {
    //     motor.setSpeed(motorRightId, 2);
    //     motor.setSpeed(motorLeftId, -2);
    // }

    // Time it.
    // while (true)
    // {
    //     setStart();
    //     motor.setSpeed(motorRightId, -2);
    //     std::cout << motor.getStatus(motorRightId).motorStatus << std::endl;

    //     double const elapsed = time();
    //     std::cout << elapsed << std::endl;
    // }

    return 0;
}


