/// \file drivers/PCA9635Driver.h
/// \brief Driver for the PCA9635 led driver.
///
/// \details This file implements all the functions to work with the led driver.
///    \note     All functions in this header should be prefixed with ledDriver_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef PCA9635_DRIVER
    #define PCA9635_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"

    class PCA9635{
        public:
            PCA9635();

            /// \brief Init and test communication with driver.
            /// \return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x40, bool const& outputInverted = false);

            /// \brief Set led brightness.
            /// \param[in] led Led number (0-15)
            /// \param[in] brightness Led brightness, from 0 to 1;
            void setDutyCycle(int const& led, double const& brightness);

        private:
            void setPinState(int const& pin, uint8_t const& ledState);

            I2CAdapter *adapter_ = nullptr;       ///< I2C port file descriptor.
            unsigned char address_ = 0;    ///< Led driver address.

            bool isInit_ = false;   ///< Status of initialization.
            unsigned char ledState_[4] = {0, 0, 0, 0};    ///< Current state of the leds: on, off or under PWM control.
    };
#endif
