/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/I2C-Wrapper.h"

#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <iostream>
#include <cstring>

bool i2c_open(I2CAdapter *adapter, std::string const& portName)
{
    adapter->file = open(portName.c_str(), O_RDWR);
    if(adapter->file < 0)
    {
        #ifdef DEBUG
            std::cout << "Failed to open i2c bus " << portName << ": " << std::strerror(errno) << std::endl;
        #endif
        return false;
    }
    // Set timeout at 10ms
    // TODO check that this has an effect.
    ioctl(adapter->file, I2C_RETRIES, 1);
    ioctl(adapter->file, I2C_TIMEOUT, 1);
    return true;
}


void changeSlave(int file, unsigned char address)
{
    int result = ioctl(file, I2C_SLAVE, address);
    #ifdef DEBUG
        if(result < 0)
            std::cout << "I2C: failed to talk to slave " << address << ": " << std::strerror(errno) << std::endl;
    #endif
}


bool i2c_writeRegister(I2CAdapter *adapter, unsigned char const& address, unsigned char const& reg, unsigned char const& data)
{
    unsigned char d = data;
    return i2c_writeRegisters(adapter, address, reg, 1, &d);
}


bool i2c_writeRegisters(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress, int const& length, unsigned char const *values)
{
    if(adapter->file < 0)
    {
        #ifdef DEBUG
            std::cout << "Error writing to I2C port: invalid file descriptor." << std::endl;
        #endif
        return false;
    }
    int messageLength = 1 + length;
    unsigned char txbuf[messageLength];
    txbuf[0] = registerAddress;
    for(int i = 0; i < length; i++)
    {
        txbuf[i + 1] = values[i];
    }

    adapter->portMutex.lock();
    changeSlave(adapter->file, address);
    int result = write(adapter->file, txbuf, messageLength);
    adapter->portMutex.unlock();
    if(result != messageLength)
    {
        #ifdef DEBUG
            std::cout << "Error writing to slave " << address << ": " << std::strerror(errno) << std::endl;
        #endif
        return false;
    }
    return true;
}

unsigned char i2c_readRegister(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress)
{
    unsigned char registerValue;
    i2c_readRegisters(adapter, address, registerAddress, 1, &registerValue);
    return registerValue;
}


bool i2c_readRegisters(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress, int const& length, unsigned char *output)
{
    if(adapter->file < 0)
    {
        #ifdef DEBUG
            std::cout << "Error reading from I2C port: invalid file descriptor." << std::endl;
        #endif
        return false;
    }
    bool returnValue = true;
    adapter->portMutex.lock();
    changeSlave(adapter->file, address);
    int result = write(adapter->file, &registerAddress, 1);
    if(result < 0)
    {
        #ifdef DEBUG
            std::cout << "Error writing to slave " << address << ": " << std::strerror(errno) << std::endl;
        #endif
        returnValue = false;
    }
    result = read(adapter->file, output, length);
    if(result < 0)
    {
        #ifdef DEBUG
            std::cout << "Error reading from slave " << address << ": " << std::strerror(errno) << std::endl;
        #endif
        returnValue = false;
    }
    adapter->portMutex.unlock();

    return returnValue;
}

void i2c_close(int device)
{
    if(device < 0)
        return;
    close(device);
}
