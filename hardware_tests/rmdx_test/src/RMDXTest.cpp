    /// A simple test for the RMDX motors

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/MCP2515Driver.h>
#include <miam_utils/drivers/RMDX.h>
#include <miam_utils/drivers/RMDXController.h>
#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/PID.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <cmath>

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

    SPIWrapper spi(RPI_SPI_00, 1000000);
    MCP2515 mcp(&spi);
    if (!mcp.init())
    {
        std::cout << "Failed to talk to MCP" << std::endl;
        exit(0);
    }

    RMDX motor(&mcp);
    int const motorRightId = 1;
    int const motorLeftId = 2;

    while (!motor.init(motorRightId))
    {
        std::cout << "Failed to init right motor, id " << motorRightId << std::endl;
        // return -1;
    }
    while (!motor.init(motorLeftId))
    {
        std::cout << "Failed to init left motor, id " << motorLeftId << std::endl;
        // return -1;
    }

    double Kp = 0.7;
    double Ki = 0.9;
    double maxOutput = 5.0;
    double filterCutoff = 10.0;
    double maxFeedforward = 0.4;
    double maxAcceleration = 2.0;

    RMDXController rightController(&motor, motorRightId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, maxAcceleration);
    RMDXController leftController(&motor, motorLeftId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, maxAcceleration);

    Logger log;
    log.start("test.hdf5");

    setStart();

    while (time() < 4)
    {
        motor.setCurrent(motorRightId, 0);
        motor.setCurrent(motorLeftId, 0);
    }

    int i = 0;
    double rightTarget, leftTarget;

    Metronome m(10000000);

    double oldt;
    oldt = m.getElapsedTime();;

    while (true)
    {
        m.wait();
        double t = m.getElapsedTime();
        double dt = t - oldt;
        oldt = t;


        // while (t > 10)
        // {
        //     t -= 10;
        // }
        rightTarget = 2 * std::sin(2 * 3.14159 * 0.25 * t);
        // if (t > 8)
        //     rightTarget = 0.0;
        leftTarget = -rightTarget;


        // rightController.sendTarget(rightTarget);
        double rightSpeed  = 0;
        double leftSpeed  = 0;
        // if (t > 8)
        // {
        //     rightController.stop();
        //     leftController.stop();
        // }
        // else
        // {

        rightSpeed = rightController.sendTarget(rightTarget, dt);
        leftSpeed = leftController.sendTarget(leftTarget, dt);
        // }


        log.log("rightTargetSpeed", time(), rightTarget);
        log.log("rightSpeed", time(), rightSpeed);
        log.log("rightPosition", time(), rightController.position_);

        log.log("rightTargetCurrent", time(), rightController.targetCurrent_);
        log.log("rightCurrent", time(), rightController.current_);

        log.log("leftTargetSpeed", time(), leftTarget);
        log.log("leftSpeed", time(), leftSpeed);
        log.log("leftPosition", time(), leftController.position_);

        log.log("leftTargetCurrent", time(), leftController.targetCurrent_);
        log.log("leftCurrent", time(), leftController.current_);


        log.log("dt", time(), dt);
    }

    // Stop
    rightController.stop();
    leftController.stop();

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


