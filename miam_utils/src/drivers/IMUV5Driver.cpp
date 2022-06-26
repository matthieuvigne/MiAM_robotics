/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/IMUV5Driver.h"
#include <unistd.h>
#include <cmath>
#include <iostream>

IMUV5::IMUV5():
    isInit_(false)
{
    // Empty
}

bool IMUV5::init(I2CAdapter *device, bool turnOnMagneto, bool jumperShorted)
{
    adapter_ = device;
    if (jumperShorted)
    {
        gyroAddress_ = 0b01101010;
        magnetoAddress_ = 0b00011100;
    }
    else
    {
        gyroAddress_ = 0b01101011;
        magnetoAddress_ = 0b00011110;
    }
    // Configure gyroscope and accelerometer.
    // Check WHO AM I
    if (i2c_readRegister(adapter_, gyroAddress_, 0x0F) != 0x69)
        return false;

    // Reset
    i2c_writeRegister(adapter_, gyroAddress_, 0x12, 0b10000001);
    usleep(20000);

    // CTRL1 : accelerometer ODR 833 Hz, 2g range, anti-aliasing bandwidth 400Hz.
    i2c_writeRegister(adapter_, gyroAddress_, 0x10, 0b01110000);
    accelScaling_ = 0.061 / 1000.0 * 9.81;
    // CTRL2: gyro ODR 833Hz, 250dps scale
    i2c_writeRegister(adapter_, gyroAddress_, 0x11, 0b01110000);
    gyroScaling_ = 0.00875 * M_PI / 180.0;
    // CTRL3: enable BDU
    i2c_writeRegister(adapter_, gyroAddress_, 0x12, 0b01000100);
    // CTRL4: accel anti-aliasing configured by CTRL1
    i2c_writeRegister(adapter_, gyroAddress_, 0x13, 0b10000000);

    // CTRL7: gyroscope high-pass filter disabled.
    i2c_writeRegister(adapter_, gyroAddress_, 0x16, 0b00000000);
    // CTRL8: disable accelerometer filters
    i2c_writeRegister(adapter_, gyroAddress_, 0x17, 0b00000000);

    // Configure magnetometer.
    if (turnOnMagneto)
    {
        // Check WHO AM I
        if (i2c_readRegister(adapter_, magnetoAddress_, 0x0F) != 0x3D)
            return false;
        // CTRL1: ultra high performance, ODR: 80Hz
        // CTRL1: enable temperature, high perf, max ODR
        i2c_writeRegister(adapter_, magnetoAddress_, 0x20, 0b11010010);
        // CTRL2: 16gauss range
        i2c_writeRegister(adapter_, magnetoAddress_, 0x21, 0b01100000);
        magnetoScaling_ = 0.0005844535359438924;
        // CTRL3: turn on
        i2c_writeRegister(adapter_, magnetoAddress_, 0x22, 0b00000000);
        // CTRL4: Z axis
        i2c_writeRegister(adapter_, magnetoAddress_, 0x23, 0b00001000);
        // CTRL5: BDU
        i2c_writeRegister(adapter_, magnetoAddress_, 0x24, 0b01000000);
    }

    // Leave some time for the gyro to fully start
    usleep(50000);


    isInit_ = true;
    return true;
}

vector3D IMUV5::getGyroscopeReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, gyroAddress_, 0x22, 6, rxbuf);
        readings.x = rxbuf[0] + (rxbuf[1] << 8);
        if(readings.x > 32767)
            readings.x -= 65536;
        readings.y = rxbuf[2] + (rxbuf[3] << 8);
        if(readings.y > 32767)
            readings.y -= 65536;
        readings.z = rxbuf[4] + (rxbuf[5] << 8);
        if(readings.z > 32767)
            readings.z -= 65536;
        readings.x *= gyroScaling_;
        readings.y *= gyroScaling_;
        readings.z *= gyroScaling_;
    }
    return readings;
}

vector3D IMUV5::getAccelerometerReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, gyroAddress_, 0x28, 6, rxbuf);
        readings.x = rxbuf[0] + (rxbuf[1] << 8);
        if(readings.x > 32767)
            readings.x -= 65536;
        readings.y = rxbuf[2] + (rxbuf[3] << 8);
        if(readings.y > 32767)
            readings.y -= 65536;
        readings.z = rxbuf[4] + (rxbuf[5] << 8);
        if(readings.z > 32767)
            readings.z -= 65536;
        readings.x *= accelScaling_;
        readings.y *= accelScaling_;
        readings.z *= accelScaling_;
    }

    return readings;
}

void IMUV5::getData(vector3D& gyro, vector3D& accel)
{
    if (isInit_)
    {
        unsigned char rxbuf[12];
        i2c_readRegisters(adapter_, gyroAddress_, 0x22, 12, rxbuf);
        gyro.x = static_cast<int16_t>(rxbuf[0] + (rxbuf[1] << 8)) * gyroScaling_;
        gyro.y = static_cast<int16_t>(rxbuf[2] + (rxbuf[3] << 8)) * gyroScaling_;
        gyro.z = static_cast<int16_t>(rxbuf[4] + (rxbuf[5] << 8)) * gyroScaling_;

        accel.x = static_cast<int16_t>(rxbuf[6] + (rxbuf[7] << 8)) * accelScaling_;
        accel.y = static_cast<int16_t>(rxbuf[8] + (rxbuf[9] << 8)) * accelScaling_;
        accel.z = static_cast<int16_t>(rxbuf[10] + (rxbuf[11] << 8)) * accelScaling_;
    }
}

vector3D IMUV5::getMagnetometerReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, magnetoAddress_, 0x28, 6, rxbuf);
        readings.x = rxbuf[0] + (rxbuf[1] << 8);
        if(readings.x > 32767)
            readings.x -= 65536;
        readings.y = rxbuf[2] + (rxbuf[3] << 8);
        if(readings.y > 32767)
            readings.y -= 65536;
        readings.z = rxbuf[4] + (rxbuf[5] << 8);
        if(readings.z > 32767)
            readings.z -= 65536;
        readings.x *= magnetoScaling_;
        readings.y *= magnetoScaling_;
        readings.z *= magnetoScaling_;
    }

    return readings;
}


double IMUV5::getMagnetometerTemperature()
{
    double readings = 0.0;
    if (isInit_)
    {
        unsigned char rxbuf[2];
        i2c_readRegisters(adapter_, magnetoAddress_, 0x2E, 2, rxbuf);
        readings = rxbuf[0] + (rxbuf[1] << 8);
        if(readings > 32767)
            readings -= 65536;
        readings = 25 + readings / 8;
    }

    return readings;
}
