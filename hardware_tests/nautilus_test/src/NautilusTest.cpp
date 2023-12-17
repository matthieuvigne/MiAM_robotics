    /// A simple test for the RMDX motors

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/NautilusWrapper.h>
#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
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
    return elapsed;
}

int main (int argc, char *argv[])
{
    RPi_enablePorts();

    NautilusWrapper rightMotor(RPI_SPI_00), leftMotor(RPI_SPI_01);

    bool encoderState;
    if (!rightMotor.init(encoderState))
    {
        std::cout << "Failed to talk to right motor" << std::endl;
        exit(0);
    }
    if (!encoderState)
        std::cout << "Warning: right encoder invalid" << std::endl;
    if (!leftMotor.init(encoderState))
    {
        std::cout << "Failed to talk to left motor" << std::endl;
        exit(0);
    }
    if (!encoderState)
        std::cout << "Warning: left encoder invalid" << std::endl;

    Logger log;
    log.start("nautilus_test.hdf5");
    setStart();

    Metronome m(2000000);

    double oldt;
    oldt = m.getElapsedTime();;

    while (time() < 50)
    {
        m.wait();
        double t = m.getElapsedTime();
        double dt = t - oldt;
        oldt = t;

        NautilusMeasurements meas = rightMotor.updateMeasurements();
        log.log("rightMotor.encoderPosition", t, meas.encoderPosition);
        log.log("rightMotor.motorVelocity", t, meas.motorVelocity);
        log.log("rightMotor.motorCurrent", t, meas.motorCurrent);
        log.log("rightMotor.batteryVoltage", t, meas.batteryVoltage);
        log.log("rightMotor.currentMode", t, meas.currentMode);
        log.log("rightMotor.nCommunicationFailed", t, meas.nCommunicationFailed);
        log.log("rightMotor.commTime", t, m.getElapsedTime() - t);

        meas = leftMotor.updateMeasurements();
        log.log("leftMotor.encoderPosition", t, meas.encoderPosition);
        log.log("leftMotor.motorVelocity", t, meas.motorVelocity);
        log.log("leftMotor.motorCurrent", t, meas.motorCurrent);
        log.log("leftMotor.batteryVoltage", t, meas.batteryVoltage);
        log.log("leftMotor.currentMode", t, meas.currentMode);
        log.log("leftMotor.nCommunicationFailed", t, meas.nCommunicationFailed);
        log.log("leftMotor.commTime", t, m.getElapsedTime() - t);


        if (time() > 10)
            setStart();

        double rightTarget = 20 * std::sin(2 * 3.14159 * 0.25 * t);
        double leftTarget = -rightTarget;

        if (time() < 8)
        {
            rightMotor.setTargetVelocity(rightTarget);
            leftMotor.setTargetVelocity(leftTarget);
        }
        else
        {
            rightTarget = 0;
            leftTarget = 0;
            rightMotor.stop();
            leftMotor.stop();
        }

        log.log("rightMotor.targetVelocity", t, rightTarget);
        log.log("leftMotor.targetVelocity", t, leftTarget);

        log.log("totalTime", t, m.getElapsedTime() - t);
        log.log("dt", t, dt);
    }


    return 0;
}


