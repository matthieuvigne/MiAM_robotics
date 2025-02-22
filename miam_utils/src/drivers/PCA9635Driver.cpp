/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/PCA9635Driver.h"

#include <iostream>

#define MODE1 0x00
#define MODE2 0x01
#define ALLCALLADR 0x1B
#define LEDOUTOFFSET 0x14
#define PWMOFFSET 2

PCA9635::PCA9635()
{

}


bool PCA9635::init(I2CAdapter *adapter, unsigned char const& address, bool const& inverted)
{
    if(adapter->file < 0)
        return false;
    adapter_ = adapter;
    address_ = address;

    isInit_ = i2c_readRegister(adapter_, address_, ALLCALLADR) == 0b11100000;
    if (isInit_)
    {
        i2c_writeRegister(adapter_, address_, MODE2, 0b100 | (inverted ? 0b10000 : 0));

        // At startup, all leds are turned off.
        for(int i = 0; i < 4; i++)
        {
            ledState_[i] = 0;
            i2c_writeRegister(adapter_, address_, LEDOUTOFFSET + i, ledState_[i]);
        }
        // Enable driver
        isInit_ = i2c_writeRegister(adapter_, address_, MODE1, 0x80);
    }
    return isInit_;
}


void PCA9635::setDutyCycle(int const& led, double const& brightness)
{
    if (isInit_ && led >= 0 && led < 16)
    {
        uint8_t value = static_cast<uint8_t>(brightness * 255);

        if(value == 0)
            setPinState(led, 0);
        else if(value == 255)
            setPinState(led, 1);
        else
        {
            i2c_writeRegister(adapter_, address_, PWMOFFSET + led, value);
            setPinState(led, 2);
        }
    }
}



void PCA9635::setPinState(int const& pin, uint8_t const& ledState)
{
    if(pin > 15 || pin < 0)
        return;

    uint8_t const lastpinstate = ledState_[pin / 4];
    ledState_[pin / 4] &= ~(1 << (2 * (pin % 4)));
    ledState_[pin / 4] &= ~(1 << (2 * (pin % 4) + 1));
    ledState_[pin / 4] |= ledState << (2 * (pin % 4));

    if(lastpinstate != ledState_[pin / 4])
    {
        i2c_writeRegister(adapter_, address_, LEDOUTOFFSET + pin / 4, ledState_[pin / 4]);
    }
}
