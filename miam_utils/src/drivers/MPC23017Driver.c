/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/drivers/MPC23017Driver.h"
#include <stdio.h>

bool mpc_init(MPC *mpc, I2CAdapter *adapter, unsigned int const& address)
{
    if(adapter->file < 0)
        return false;
    mpc->adapter = adapter;
    mpc->address=address;
    // Test communication and chip.
    // This chip has no who-a-i register, so we will just check we get the right feedback after a write.

    // Reset IOCON configuration register, in both bank configuration.
    i2c_writeRegister(mpc->adapter, mpc->address, 0x0B, 0x00);
    i2c_writeRegister(mpc->adapter, mpc->address, 0x15, 0x00);
    // Set all pins as input.
    i2c_writeRegister(mpc->adapter, mpc->address, 0x00, 0xFF);
    i2c_writeRegister(mpc->adapter, mpc->address, 0x01, 0xFF);
    // Reset everything
    for(int reg = 2; reg < 20; reg++)
        i2c_writeRegister(mpc->adapter, mpc->address, reg, 0x00);

    // Enable pullup on all ports.
    i2c_writeRegister(mpc->adapter, mpc->address, 0x0C, 0xFF);
    i2c_writeRegister(mpc->adapter, mpc->address, 0x0D, 0xFF);

    // Chech writing took effect (i.e chip is there, we hope it's the right one).
    if(i2c_readRegister(mpc->adapter, mpc->address, 0x00) != 0xFF)
    {
        #ifdef DEBUG
            printf("Error : MPC23017 (IO expander) not detected\n");
        #endif
        return false;
    }
    return true;
}

void mpc_pinMode(MPC mpc, unsigned int pin, MPCPinState state)
{
    if(pin < 0 || pin > 15)
        return;
    if(state < 0 || state > 2)
        return;
    int port = 0;
    if(pin > 7)
    {
        pin -=8;
        port = 1;
    }
    unsigned char currentState = i2c_readRegister(mpc.adapter, mpc.address, 0x00 + port);
    if(state == 2)
        i2c_writeRegister(mpc.adapter, mpc.address, 0x00 + port, currentState & ~(1 << pin));
    else
    {
        unsigned char pullupState = i2c_readRegister(mpc.adapter, mpc.address, 0x0C + port);
        i2c_writeRegister(mpc.adapter, mpc.address, 0x00 + port, currentState | (1 << pin));
        if(state == 1)
            i2c_writeRegister(mpc.adapter, mpc.address, 0x0C + port, pullupState | (1 << pin));
        else
            i2c_writeRegister(mpc.adapter, mpc.address, 0x0C + port, pullupState & ~(1 << pin));
    }
}

int mpc_digitalRead(MPC mpc, unsigned int pin)
{
    if(pin < 0 || pin > 15)
        return 0;
    int port = 0;
    if(pin > 7)
    {
        pin -=8;
        port = 1;
    }
    unsigned char currentState = i2c_readRegister(mpc.adapter, mpc.address, 0x12 + port);
    if((currentState & (1 << pin)) > 0)
        return 1;
    else
        return 0;
}

void mpc_digitalWrite(MPC mpc, unsigned int pin, unsigned int const& value)
{
    if(pin < 0 || pin > 15)
        return;
    int port = 0;
    if(pin > 7)
    {
        pin -=8;
        port = 1;
    }
    unsigned char currentState = i2c_readRegister(mpc.adapter, mpc.address, 0x12 + port);
    if(value == 0)
        i2c_writeRegister(mpc.adapter, mpc.address, 0x12 + port, currentState & ~(1 << pin));
    else
        i2c_writeRegister(mpc.adapter, mpc.address, 0x12 + port, currentState | (1 << pin));
}

void mpc_writeAll(MPC mpc, unsigned int const& value)
{
    i2c_writeRegister(mpc.adapter, mpc.address, 0x12, value & 0xFF);
    i2c_writeRegister(mpc.adapter, mpc.address, 0x13, (value >> 8));
}

unsigned int mpc_readAll(MPC mpc)
{
    unsigned char abValue[2];
    i2c_readRegisters(mpc.adapter, mpc.address, 0x12, 2, abValue);
    return (abValue[1] << 8) + abValue[0];
}
