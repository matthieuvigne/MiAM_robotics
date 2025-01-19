#include "miam_utils/drivers/PCA9546ADriver.h"

#include <iostream>

#define MODE1 0x00
#define MODE2 0x01
#define ALLCALLADR 0x1B
#define LEDOUTOFFSET 0x14
#define PWMOFFSET 2

PCA9546A::PCA9546A()
{

}


bool PCA9546A::init(I2CAdapter *adapter, unsigned char const& address)
{
    if(adapter->file < 0)
        return false;
    adapter_ = adapter;
    address_ = address;

    setPorts(0);
    return getPorts() == 0;
}


void PCA9546A::setPorts(unsigned char const& ports)
{
    i2c_writeRegister(adapter_, address_, 0x00, ports);
}



unsigned char PCA9546A::getPorts()
{
    return i2c_readRegister(adapter_, address_, 0x00);
}