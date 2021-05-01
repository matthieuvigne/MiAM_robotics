
#include "miam_utils/drivers/PCA9685Driver.h"

PCA9685::PCA9685():
    adapter_(nullptr),
    address_(0),
    isInit_(false)
{

}

bool PCA9685::init(I2CAdapter *device, unsigned char const& address)
{
    adapter_ = device;
    address_ = address;

    // Configuration: enable oscillator, set correct logic (no external drivers)
    i2c_writeRegister(adapter_, address_, 0x00, 0b00100000);
    i2c_writeRegister(adapter_, address_, 0x01, 0b00010110);

    // Set prescaler:
    i2c_writeRegister(adapter_, address_, 0xFE, 0x03);

    // Reset all
    unsigned char values[4] = {0x00, 0x00, 0x00, 0x10};
    i2c_writeRegisters(adapter_, address_, 0xFA, 4, values);

    // Test back communication
    isInit_ = i2c_readRegister(adapter_, address_, 0x01) == 0b00010110;
    return isInit_;
}


void PCA9685::setBrightness(int const& led, double const& brightness)
{
    if (isInit_ && led >= 0 && led < 16)
    {
        // We only write 3 bytes since the first one is always set to zero (no delay)
        unsigned char register_values[3] = {0};
        if(brightness < 0)
        {
            // Full on.
            register_values[2] = 0x10;
        }
        else if (brightness > 1)
        {
            // Full on.
            register_values[0] = 0x10;
        }
        else
        {
            // Convert to int
            int value = static_cast<int>(brightness * 4096);
            register_values[1] = value & 0xFF;
            register_values[2] = (value >> 8) & 0x0F;
        }
        i2c_writeRegisters(adapter_, address_, 0x07 + 4 * led, 3, register_values);
    }

}