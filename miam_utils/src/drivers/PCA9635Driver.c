/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/drivers/PCA9635Driver.h"

#define LEDOUTOFFSET 0x14
#define PWMOFFSET 2

void setPinState(PCA9635 *d, int pin, char ledState)
{
    // Change a single pin state.
    if(pin > 15 || pin < 0) return;

    int lastpinstate = d->ledState[pin / 4];
    d->ledState[pin / 4] &= ~(1 << (2 * (pin % 4)));
    d->ledState[pin / 4] &= ~(1 << (2 * (pin % 4) + 1));
    d->ledState[pin / 4] |= ledState << (2 * (pin % 4));

    if(lastpinstate != d->ledState[pin / 4])
        i2c_writeRegister(d->adapter, d->address, LEDOUTOFFSET + pin / 4, d->ledState[pin / 4]);
}


bool ledDriver_init(PCA9635 *d, I2CAdapter *adapter, unsigned char address)
{
    if(adapter->file < 0)
        return false;
    d->adapter = adapter;
    d->address = address;
    // At startup, all leds are turned off.
    d->ledState[0] = 255;
    d->ledState[2] = 255;
    d->ledState[1] = 255;
    d->ledState[3] = 255;
    for(int x = 0; x < 16; x ++)
        ledDriver_setLedBrightness(d, x, 0);
    // Enable driver
    return i2c_writeRegister(d->adapter, d->address, 0x80, 0x80);
}


void ledDriver_setLedBrightness(PCA9635 *d, int pin, int brightness)
{
    if(d->adapter < 0)
        return;
    if(pin > 15 || pin < 0) return;
    if(brightness < 0)     brightness = 0;
    if(brightness > 255)   brightness = 255;

    if(brightness == 0)
        setPinState(d, pin, 0);
    else if(brightness == 255)
        setPinState(d, pin, 1);
    else
    {
        setPinState(d, pin, 2);
        i2c_writeRegister(d->adapter, d->address, PWMOFFSET + pin, brightness);
    }
}


void ledDriver_setRGBledBrightness(PCA9635 *d, int led, int r, int g, int b)
{
    switch(led)
    {
        case 1: ledDriver_setLedBrightness(d,1,r);
                ledDriver_setLedBrightness(d,0,g);
                ledDriver_setLedBrightness(d,2,b); break;
        case 2: ledDriver_setLedBrightness(d,13,r);
                ledDriver_setLedBrightness(d,14,g);
                ledDriver_setLedBrightness(d,12,b); break;
        case 3: ledDriver_setLedBrightness(d,4,r);
                ledDriver_setLedBrightness(d,3,g);
                ledDriver_setLedBrightness(d,5,b); break;
    }
}
