/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/SparkfunOdometryDriver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    SparkfunOdometry sensor;

    if (!sensor.init(&RPI_I2C))
    {
        std::cout << "Failed to init SparkfunOdometry" << std::endl;
        return -1;
    }

    Metronome metronome(0.100 * 1e9);

    Logger logger;
    logger.start("SparkfunOdometrytest.data");

    while (true)
    {

        metronome.wait();
        double const currentTime = metronome.getElapsedTime();

        miam::RobotPosition pos = sensor.read();
        logger.log("posX", currentTime, pos.x);
        logger.log("posY", currentTime, pos.y);
        logger.log("posTheta", currentTime, pos.theta);
        std::cout << "\r" << std::setw(10) << std::setfill(' ') << std::setprecision(5) << pos << std::flush;
    }

    return 0;
}


