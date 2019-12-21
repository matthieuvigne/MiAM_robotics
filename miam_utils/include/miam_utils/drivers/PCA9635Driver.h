/// \file drivers/PCA9635Driver.h
/// \brief Driver for the PCA9635 led driver.
///
/// \details This file implements all the functions to work with the led driver.
///    \note     All functions in this header should be prefixed with ledDriver_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef PCA9635_DRIVER
    #define PCA9635_DRIVER
    #include "MiAMEurobot/drivers/I2C-Wrapper.h"

    /// Led driver structure.
    typedef struct {
        I2CAdapter *adapter;        ///< I2C port file descriptor.
        int address;    ///< Led driver address.
        unsigned char ledState[4];    ///< Current state of the leds: on, off or under PWM control.
    }PCA9635;


    /// \brief Initialize Led driver.
    ///
    /// \details This function tests the communication with the led driver, and, if successful, inits the structure.
    ///
    /// \param[out] driver The PCA9635 structure, to be used whenever communication with the led driver.
    /// \param[in] adapter Pointer to a valid I2CAdapter to choose the I2C port (as returned by the i2c_open function,
    ///                    see I2C-Wrapper.h).
    /// \param[in] address I2C address of the PCA9635.
    /// \returns   true on success, false otherwise.
    bool ledDriver_init(PCA9635 *driver, I2CAdapter *adapter, unsigned char address);

    /// \brief Set brightness of a single LED.
    ///
    /// \param[in,out] driver The PCA9635 structure (as a pointer as the ledState variable might be modified).
    /// \param[in] pin The number of the pin to change (from 0 to 15, see component datasheet).
    /// \param[in] brightness Led brightness. 0 turns the led 0, 255 is the maximum value.
    void ledDriver_setLedBrightness(PCA9635 *driver, int pin, int brightness);

    /// \brief Set brightness of a RGB LED.
    ///    \note This is equivalent to three successive calls of ledDriver_setLedBrightness, but it uses
    ///          predefined (hardcoded) constants to match the RGB leds on the output PCB.
    ///
    /// \param[in,out] driver The PCA9635 structure.
    /// \param[in] pin The number of RGB led (from 1 to 3).
    /// \param[in] r Red led brightness. 0 turns the led 0, 255 is the maximum value.
    /// \param[in] g Green led brightness. 0 turns the led 0, 255 is the maximum value.
    /// \param[in] b Blue led brightness. 0 turns the led 0, 255 is the maximum value.
    void ledDriver_setRGBledBrightness(PCA9635 *driver, int led, int r, int g, int b);
#endif
