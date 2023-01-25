    /// A simple test for the RMDX motors

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/MCP2515Driver.h>
#include <miam_utils/drivers/RMDX.h>
#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/PID.h>
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
    RPi_enablePorts();

    SPIWrapper spi(RPI_SPI_00, 300000);
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

    Logger log;
    log.start("test.hdf5");

    setStart();

    while (time() < 6)
    {
        motor.setCurrent(motorRightId, 0);
        motor.setCurrent(motorLeftId, 0);
    }

    int i = 0;
    double rightTarget, leftTarget;

    Metronome m(2000000);

    double oldt;
    oldt = m.getElapsedTime();;

    while (true)
    {
        while (time() / 2 > i)
            i++;
        if (i % 2)
        {
            rightTarget = 2.0;
            leftTarget = -2.0;
        }
        else
        {
            rightTarget = -2.0;
            leftTarget = 3.0;
        }
        leftTarget = -rightTarget;

        m.wait();
        double t = m.getElapsedTime();
        double dt = t - oldt;
        oldt = t;

        double rightSpeed = motor.setSpeed(motorRightId, rightTarget);
        double leftSpeed = motor.setSpeed(motorLeftId, leftTarget);

        log.log("rightTargetSpeed", time(), rightTarget);
        log.log("rightSpeed", time(), rightSpeed);
        log.log("leftTargetSpeed", time(), leftTarget);
        log.log("leftSpeed", time(), leftSpeed);

        log.log("dt", time(), dt);
    }

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


