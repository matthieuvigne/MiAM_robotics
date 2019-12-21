/// \file drivers/TCS3472ColorSensorDriver.h
/// \brief Driver for TCS3472x RGB/clear light sensor.
///
/// \details This file implements all the functions to work with the TCS34725, mounted on Adafruit ADA1334.
///          This code will also work for TCS34727 - for TCS34721 and TCS34723 slave address must be changed
///          from 0x29 to 0x39 in the source code.
///    \note     All functions in this header should be prefixed with colorSensor_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef TCS3472_DRIVER
    #define TCS3472_DRIVER
    #include "MiAMEurobot/drivers/I2C-Wrapper.h"

    /// Color sensor structure.
    typedef struct {
        I2CAdapter *adapter;        ///< I2C port file descriptor.
    }ColorSensorTCS3472;

    ///< Enum giving the list of possible ADC gain.
    typedef enum
    {
      TCS34725_GAIN_1X                = 0x00,   ///<  No gain
      TCS34725_GAIN_4X                = 0x01,   ///<  4x gain
      TCS34725_GAIN_16X               = 0x02,   ///<  16x gain
      TCS34725_GAIN_60X               = 0x03    ///<  60x gain
    }
    TCS34725Gain_t;

    typedef struct{
        int red;
        int green;
        int blue;
        int clear;
    }ColorOutput;

    /// \brief Initialize the sensor driver.
    ///
    /// \details This function tests the communication with the sensor, and, if successful, inits the structure.
    ///          It also sets the gain to 1, and the integration time to 3ms.
    ///
    /// \param[out] driver The ColorSensorTCS3472 structure, to be used whenever communication with the sensor.
    /// \param[in] adapter Pointer to a valid I2CAdapter to choose the I2C port (as returned by the i2c_open function,
    ///                    see I2C-Wrapper.h).
    /// \returns   true on success, false otherwise.
    bool colorSensor_init(ColorSensorTCS3472 *sensor, I2CAdapter *adapter);

    /// \brief Set sensor integration time.
    ///
    /// \details This color sensor integrates data from the ADC before returning a digital readinig. This function
    ///          sets this integration time, which therefore also corresponds to the update period of the chip.
    ///
    /// \param[in] sensor The ColorSensorTCS3472 structure.
    /// \param[in] integrationTime Duration of integration, in ms. Values are clamped between 2.4ms and 700ms,
    ///            in 2.4ms increment.
    /// \returns   true on success, false otherwise.
    bool colorSensor_setIntegrationTime(ColorSensorTCS3472 sensor, int integrationTime);

    /// \brief Set ADC gain.
    ///
    /// \param[in] sensor The ColorSensorTCS3472 structure.
    /// \param[in] gain The gain of the ADC, as an element of the TCS34725Gain_t enum.
    /// \returns   true on success, false otherwise.
    bool colorSensor_setGain(ColorSensorTCS3472 sensor, TCS34725Gain_t gain);

    /// \brief Get sensor data.
    ///
    /// \param[in] sensor The ColorSensorTCS3472 structure.
    /// \returns   A ColorOutput structure contining red, green, blue and clear data (16-bit resolution).
    ColorOutput colorSensor_getData(ColorSensorTCS3472 sensor);
#endif
