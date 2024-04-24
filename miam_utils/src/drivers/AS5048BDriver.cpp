#include "miam_utils/drivers/AS5048BDriver.h"

#include <iostream>

AS5048B::AS5048B():
    adapter_(nullptr),
    address_(0),
    isInit_(false)
{

}

bool AS5048B::init(I2CAdapter *device, unsigned char const& address)
{
    adapter_ = device;
    address_ = address;

    unsigned char rxbuf[6];
    isInit_ = i2c_readRegisters(adapter_, address_, 0x15, 1, rxbuf);

    return isInit_;
}

int AS5048B::getPosition()
{
    if (isInit_)
    {
        unsigned char rxbuf[6];
        i2c_readRegisters(adapter_, address_, 0xFA + 1, 6, rxbuf);

        std::cout << "AGC" << static_cast<int>(rxbuf[0]) << std::endl;
        std::cout << "DIAG" << static_cast<int>(rxbuf[1]) << std::endl;
        std::cout << "MAG" << static_cast<int>(rxbuf[2]) << std::endl;
        std::cout << "MAG" << static_cast<int>(rxbuf[3]) << std::endl;
        std::cout << "POS" << static_cast<int>(rxbuf[4]) << std::endl;
        std::cout << "POS" << static_cast<int>(rxbuf[5]) << std::endl;

        return (rxbuf[4] << 6)  + rxbuf[5];
    }
    return 0;
}