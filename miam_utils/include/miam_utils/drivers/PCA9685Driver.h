/// \file drivers/PCA9685Driver.h
/// \brief Driver for the PCA9685 led driver.
///
/// \details This file implements all the functions to work with the led driver.
///    \note     All functions in this header should be prefixed with ledDriver_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef PCA9685_DRIVER
    #define PCA9685_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"

    class PCA9685{
        public:
            PCA9685();

            /// \brief Init and test communication with driver.
            /// \return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x40);

            /// \brief Set led brightness.
            /// \param[in] led Led number (0-15)
            /// \param[in] brightness Led brightness, from 0 to 1;
            void setBrightness(int const& led, double const& brightness);


        private:
            I2CAdapter *adapter_;       ///< I2C port file descriptor.
            unsigned char address_;    ///< Led driver address.

            bool isInit_;   ///< Status of initialization.
    };
#endif
