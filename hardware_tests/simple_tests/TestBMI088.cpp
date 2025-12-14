/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/BMI088Driver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    BMI088 imu;

    if (!imu.init(&RPI_I2C) && !imu.init(&RPI_I2C, 0x18, 0x68))
    {
        std::cout << "Failed to init BMI088" << std::endl;
        return -1;
    }

    Metronome metronome(0.100 * 1e9);

    Logger logger;
    logger.start("bmi088test.data");

    while (true)
    {

        metronome.wait();
        double const currentTime = metronome.getElapsedTime();

        Eigen::Vector3f gyro = imu.getGyroscopeReadings();
        Eigen::Vector3f accel = imu.getAccelerometerReadings();
        logger.log("gyroX", currentTime, gyro(0));
        logger.log("gyroY", currentTime, gyro(1));
        logger.log("gyroZ", currentTime, gyro(2));
        logger.log("accelX", currentTime, accel(0));
        logger.log("accelY", currentTime, accel(1));
        logger.log("accelZ", currentTime, accel(2));

        std::cout << "\r" << std::setw(10) << std::setfill(' ') << std::setprecision(2) << gyro.transpose() << " " << accel.transpose() << std::endl;
    }

    return 0;
}


