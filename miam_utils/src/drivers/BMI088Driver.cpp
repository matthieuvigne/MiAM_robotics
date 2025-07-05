/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/BMI088Driver.h"
#include <unistd.h>
#include <cmath>
#include <iostream>

int16_t rawToInt(unsigned char *val)
{
    uint16_t data = (static_cast<uint16_t>(*(val + 1)) << 8) + *val;
    return static_cast<int16_t>(data);
}

BMI088::BMI088():
    isInit_(false)
{
    // Empty
}

bool BMI088::init(I2CAdapter *device, unsigned char const& accelAddress, unsigned char const& gyroAddress)
{
    adapter_ = device;
    accelAddress_ = accelAddress;
    gyroAddress_ = gyroAddress;

    // Configure gyroscope
    if (i2c_readRegister(adapter_, gyroAddress_, 0x00) != 0x0F)
        return false;

    // Range: 250dps
    i2c_writeRegister(adapter_, gyroAddress_, 0x0F, 0x03);
    gyroScaling_ = M_PI / 180.0 / 131.072;

    // Bandwidth: 400Hz ODR
    i2c_writeRegister(adapter_, gyroAddress_, 0x10, 0x03);

    // Power up
    i2c_writeRegister(adapter_, gyroAddress_, 0x11, 0x00);


    // Configure accelerometer
    if (i2c_readRegister(adapter_, accelAddress_, 0x00) != 0x1E)
        return false;

    // Range: 3G
    i2c_writeRegister(adapter_, accelAddress_, 0x41, 0x00);
    accelScaling_ = 0.00981 / 32768.0 * 1000 * 2 * 1.5;

    // Bandwidth: 400Hz ODR
    i2c_writeRegister(adapter_, accelAddress_, 0x40, (0x0A << 4) + 0x0A);

    // Power up
    i2c_writeRegister(adapter_, accelAddress_, 0x7C, 0x00);
    i2c_writeRegister(adapter_, accelAddress_, 0x7D, 0x04);

    isInit_ = true;
    return isInit_;
}

Eigen::Vector3f BMI088::getGyroscopeReadings()
{
    Eigen::Vector3f readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, gyroAddress_, 0x02, 6, rxbuf);
        for (int i = 0; i < 3; i++)
            readings(i) = static_cast<int16_t>(rxbuf[2*i] + (rxbuf[2 * i + 1] << 8)) * gyroScaling_;
    }
    return readings;
}

Eigen::Vector3f BMI088::getAccelerometerReadings()
{
    Eigen::Vector3f readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, accelAddress_, 0x12, 6, rxbuf);
        for (int i = 0; i < 3; i++)
            readings(i) = static_cast<int16_t>(rxbuf[2*i] + (rxbuf[2 * i + 1] << 8)) * accelScaling_;
    }
    return readings;
}

