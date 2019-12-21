/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/drivers/TCS3472ColorSensorDriver.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

// Slave address: constant by chip reference.
// For TCS34725 and TCS34727 use 0x29.
// For TCS34721 and TCS34723 use 0x39.
const unsigned char TCS3472_ADDRESS = 0x29;

const unsigned char TCS3472_COMMAND_BIT = 0x80;

bool colorSensor_init(ColorSensorTCS3472 *sensor, I2CAdapter *adapter)
{
    if(adapter->file < 0)
        return false;
    sensor->adapter = adapter;
    // Check chip identity.
    unsigned char chipIdentity = i2c_readRegister(sensor->adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x12);
    if(chipIdentity != 0x44 && chipIdentity != 0x4D)
    {
        #ifdef DEBUG
            printf("Error : TCS3472 not detected\n");
        #endif
        return false;
    }
    // Configure chip.
    // Set command register
    i2c_writeRegister(sensor->adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x00, 0b10100000);

    // Set default integration time and gain.s
    colorSensor_setIntegrationTime(*sensor, 3);
    colorSensor_setGain(*sensor, TCS34725_GAIN_1X);
    // Enable chip.
    i2c_writeRegister(sensor->adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x00, 0x01);
    // Wait 3ms for oscillator to start, then powerup the device.
    usleep(3000);
    i2c_writeRegister(sensor->adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x00, 0x03);
    return true;
}


bool colorSensor_setIntegrationTime(ColorSensorTCS3472 sensor, int integrationTime)
{
    // Compute register value.
    unsigned char registerValue = (unsigned char) (256 - (floor(integrationTime / 2.4)));
    return i2c_writeRegister(sensor.adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x01, registerValue);
}

bool colorSensor_setGain(ColorSensorTCS3472 sensor, TCS34725Gain_t gain)
{
    return i2c_writeRegister(sensor.adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x0F, gain);
}

ColorOutput colorSensor_getData(ColorSensorTCS3472 sensor)
{
    ColorOutput color;
    unsigned char sensorValue[8];
    i2c_readRegisters(sensor.adapter, TCS3472_ADDRESS, TCS3472_COMMAND_BIT | 0x14, 8, sensorValue);
    color.clear = (sensorValue[0] << 8) + sensorValue[1];
    color.red = (sensorValue[2] << 8) + sensorValue[3];
    color.green = (sensorValue[4] << 8) + sensorValue[5];
    color.blue = (sensorValue[6] << 8) + sensorValue[7];

    return color;
}
