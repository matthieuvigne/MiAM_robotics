/// \file drivers/INA226Driver.h
/// \brief Driver for the INA226 power monitor led driver.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef INA226_DRIVER
    #define INA226_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"

    struct INA226Reading{
        double voltage = 0.0; // V
        double current = 0.0; // A
        double power = 0.0;   // W
    };

    class INA226{
        public:
            INA226();

            /// \brief Init and test communication with driver.
            /// \return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x40, double const& Rshunt = 0.005);

            /// \brief Reader power statistics.
            INA226Reading read();


        private:
            I2CAdapter *adapter_;       ///< I2C port file descriptor.
            unsigned char address_;    ///< Led driver address.

            bool isInit_;   ///< Status of initialization.
            double currentLSB_ = 0.001; // Current LSB for conversion

            uint16_t readU16Register(uint8_t const& address);
            bool writeU16Register(uint8_t const& address, uint16_t const& value);
    };
#endif
