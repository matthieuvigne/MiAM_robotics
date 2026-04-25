/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/I2C-Wrapper.h"

#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <iostream>
#include <cstring>
#include <vector>

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
    if(result < 0)
    {
        #ifdef DEBUG
            std::cout << "I2C: failed to talk to slave " << address << ": " << std::strerror(errno) << std::endl;
        #endif
    }
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

    std::vector<uint8_t> buffer(1 + length);
    buffer[0] = registerAddress;
    for (int i = 0; i < length; i++)
        buffer[i + 1] = values[i];

    struct i2c_msg msg;
    msg.addr  = address;
    msg.flags = 0; // write
    msg.len   = buffer.size();
    msg.buf   = buffer.data();

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = &msg;
    ioctl_data.nmsgs = 1;

    adapter->portMutex.lock();
    int result = ioctl(adapter->file, I2C_RDWR, &ioctl_data);
    adapter->portMutex.unlock();

    if(result < 0)
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
    unsigned char registerValue = 0;
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

    struct i2c_msg msgs[2];
    msgs[0].addr  = address;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    unsigned char regAdd = registerAddress;
    msgs[0].buf   = &regAdd;

    msgs[1].addr  = address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = length;
    msgs[1].buf   = output;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    adapter->portMutex.lock();
    int result = ioctl(adapter->file, I2C_RDWR, &ioctl_data);
    adapter->portMutex.unlock();

    if(result < 0)
    {
        #ifdef DEBUG
            std::cout << "Error writing to slave " << address << ": " << std::strerror(errno) << std::endl;
        #endif
        return false;
    }
    return true;
}

void i2c_close(int device)
{
    if(device < 0)
        return;
    close(device);
}
