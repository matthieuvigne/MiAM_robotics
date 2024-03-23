
#include "miam_utils/drivers/MCP23008Driver.h"

#include <iostream>

#define IODIR_REG 0x00
#define GPIO_REG 0x09

MCP23008::MCP23008():
    adapter_(nullptr),
    address_(0),
    isInit_(false),
    currentState_(0)
{

}

bool MCP23008::init(I2CAdapter *device, unsigned char const& address)
{
    adapter_ = device;
    address_ = address;

    // Set port 6 and 7 as input, to check that the device responds correctly
    i2c_writeRegister(adapter_, address_, IODIR_REG, 0b11000000);
    isInit_ = i2c_readRegister(adapter_, address_, IODIR_REG) == 0b11000000;

    // Clear state and set everything as output
    setOutputs(0);
    i2c_writeRegister(adapter_, address_, IODIR_REG, 0);

    return isInit_;
}


void MCP23008::setPin(unsigned char const& portId, bool const& high)
{
    if (high)
        currentState_ |= (1 << portId);
    else
        currentState_ &= ~(1 << portId);
    setOutputs(currentState_);
}

void MCP23008::setOutputs(unsigned char const& bitMask)
{
    currentState_ = bitMask;
    i2c_writeRegister(adapter_, address_, GPIO_REG, currentState_);
}