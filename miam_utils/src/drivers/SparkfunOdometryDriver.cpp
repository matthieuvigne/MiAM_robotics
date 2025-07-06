
#include "miam_utils/drivers/SparkfunOdometryDriver.h"
#include <math.h>

SparkfunOdometry::SparkfunOdometry():
    adapter_(nullptr),
    address_(0)
{

}

bool SparkfunOdometry::init(I2CAdapter *device, unsigned char const& address)
{
    adapter_ = device;
    address_ = address;

    if (i2c_readRegister(adapter_, address_, 0x00) != 0x5F)
        return false;

    // Enable
    i2c_writeRegister(adapter_, address_, 0x0E, 0b1111);

    // Reset
    resetPosition(miam::RobotPosition());

    return true;
}

void SparkfunOdometry::resetPosition(miam::RobotPosition const& resetPos)
{
    // Reset sensor
    i2c_writeRegister(adapter_, address_, 0x07, 1);
    positionOffset_ = resetPos;
}

miam::RobotPosition SparkfunOdometry::read()
{
    miam::RobotPosition rawPos;

    unsigned char rxbuf[6];
    i2c_readRegisters(adapter_, address_, 0x20, 6, rxbuf);
    rawPos.x = static_cast<int16_t>(rxbuf[0] + (rxbuf[1] << 8)) / 32768.0 * 10000;
    rawPos.y = static_cast<int16_t>(rxbuf[2] + (rxbuf[3] << 8)) / 32768.0 * 10000;
    rawPos.theta = static_cast<int16_t>(rxbuf[4] + (rxbuf[5] << 8)) / 32768.0 * M_PI;

    return positionOffset_ + rawPos.rotate(positionOffset_.theta);
}
