
#include "miam_utils/drivers/INA226Driver.h"

#include <iostream>
// Calibration value
// 15A -> LSB target 0.5mA , 5mOhm:
#define TARGET_CURRENT_LSB 0.0005

double const R = 0.0050;
uint16_t const calibRegister = static_cast<uint16_t>(0.00512 / R / TARGET_CURRENT_LSB);

double const currentLSB = 0.00512 / R / calibRegister;

INA226::INA226():
    adapter_(nullptr),
    address_(0),
    isInit_(false)
{

}

bool INA226::init(I2CAdapter *device, unsigned char const& address)
{
    adapter_ = device;
    address_ = address;

    // Read device id
    isInit_ = readU16Register(0xFF) == 0x2260;
    if (isInit_)
    {
        // Configure
        writeU16Register(0x00, 0b0100010011011111); //16 value average, 588us sample,
        writeU16Register(0x05, calibRegister);
    }
    return isInit_;
}


INA226Reading INA226::read()
{
    INA226Reading readings;
    if (isInit_)
    {
        readings.voltage = readU16Register(0x02) * 0.00125;

        readings.current = static_cast<int16_t>(readU16Register(0x04)) * currentLSB;
        readings.power = readU16Register(0x03) * currentLSB * 25;
    }

    return readings;
}


uint16_t INA226::readU16Register(uint8_t const& address)
{
    uint8_t output[2];
    if (i2c_readRegisters(adapter_, address_, address, 2, output))
        return (output[0] << 8) + output[1];
    return 0;
}


bool INA226::writeU16Register(uint8_t const& address, uint16_t const& value)
{
    uint8_t values[2];
    values[0] = (value >> 8) & 0xFF;
    values[1] = value & 0xFF;
    return i2c_writeRegisters(adapter_, address_, address, 2, values);
}