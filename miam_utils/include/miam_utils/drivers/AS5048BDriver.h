#ifndef AS5048B_DRIVER_H
#define AS5048B_DRIVER_H

#include <miam_utils/drivers/I2C-Wrapper.h>
#include <vector>

class AS5048B{
    public:
        AS5048B();

        /// @brief Init communication with device
        /// @param device I2C device
        /// @param address Device address
        /// @return True on successful connection
        bool init(I2CAdapter *device, unsigned char const& address = 0x40);

        /// \brief Read position from sensor (debug registers are also read and printed in case of error)
        /// \return New position (ticks)
        int getPosition();

    private:
        I2CAdapter *adapter_;       ///< I2C port file descriptor.
        unsigned char address_;    ///< Led driver address.
        bool isInit_;   ///< Status of initialization.
};


#endif
