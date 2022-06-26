/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/IMUV3Driver.h"
#include <cmath>

IMUV3::IMUV3():
    isInit_(false)
{
    // Empty
}

bool IMUV3::init(I2CAdapter *device, bool turnOnMagneto, bool jumperShorted)
{
    adapter_ = device;
    if (jumperShorted)
    {
        gyroAddress_ = 0b1101010;
        accelAddress_ = 0b0011110;
    }
    else
    {
        gyroAddress_ = 0b1101011;
        accelAddress_ = 0b0011101;
    }
    // Configure gyroscope.
    // Check WHO AM I
    if (i2c_readRegister(adapter_, gyroAddress_, 0x0F) != 0b11010111)
        return false;

    // CTRL1 : ODR 800 Hz, bandwith 100Hz, enable all.
    i2c_writeRegister(adapter_, gyroAddress_, 0x20, 0b11111111);
    // CTRL2: high-pass at 0.05 Hz
    i2c_writeRegister(adapter_, gyroAddress_, 0x21, 0b00001000);
    // CTRL3: leave at default
    i2c_writeRegister(adapter_, gyroAddress_, 0x22, 0);
    // CTRL4: scale 245dps, enable BDU
    i2c_writeRegister(adapter_, gyroAddress_, 0x23, 0b10000000);
    // CTRL5: enable high-pass
    i2c_writeRegister(adapter_, gyroAddress_, 0x24, 0b00010011);
    gyroScaling_ = 0.00875 * M_PI / 180.0;

    // Configure accel + magnetometer.
    // Check WHO AM I
    if (i2c_readRegister(adapter_, accelAddress_, 0x0F) != 0b01001001)
        return false;
    // CTRL0: default
    i2c_writeRegister(adapter_, accelAddress_, 0x1F, 0);
    // CTRL1: accel ODR 800Hz, block update, enable
    i2c_writeRegister(adapter_, accelAddress_, 0x20, 0b10011111);
    // CTRL2: anti-aliasing 194Hz, 2g range
    i2c_writeRegister(adapter_, accelAddress_, 0x21, 0b01000000);
    accelScaling_ = 0.061 / 1000.0 * 9.81;
    // CTRL3: default
    i2c_writeRegister(adapter_, accelAddress_, 0x22, 0);
    // CTRL4: default
    i2c_writeRegister(adapter_, accelAddress_, 0x23, 0);
    // CTRL5: magneto high res, ODR 100Hz.
    i2c_writeRegister(adapter_, accelAddress_, 0x24, 0b01110100);
    // CTRL6: magneto scale 12 gauss.
    i2c_writeRegister(adapter_, accelAddress_, 0x25, 0b01100000);
    magnetoScaling_ = 0.000479;
    // CTRL7: no accel high-pass, turn on magneto if asked for.
    if (turnOnMagneto)
        i2c_writeRegister(adapter_, accelAddress_, 0x26, 0b00000000);
    else
        i2c_writeRegister(adapter_, accelAddress_, 0x26, 0b00000001);


    isInit_ = true;
    return true;
}

vector3D IMUV3::getGyroscopeReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, gyroAddress_, 0xA8, 6, rxbuf);
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

vector3D IMUV3::getAccelerometerReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, accelAddress_, 0xA8, 6, rxbuf);
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

vector3D IMUV3::getMagnetometerReadings()
{
    vector3D readings;
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, accelAddress_, 0x88, 6, rxbuf);
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

