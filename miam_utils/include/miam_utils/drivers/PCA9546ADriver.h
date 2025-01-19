/// \brief Driver for the PCA9546A I2C multiplexer.
#ifndef PCA9546A_DRIVER
    #define PCA9546A_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"

    class PCA9546A{
        public:
            PCA9546A();

            /// \brief Init and test communication with driver.
            /// \return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x71);

            /// \brief Set I2C port states
            /// \param[in] ports Binary mask of enabled ports
            void setPorts(unsigned char const& ports);

            /// \brief Return currently enabled ports
            unsigned char getPorts();

        private:
            I2CAdapter *adapter_ = nullptr;
            unsigned char address_ = 0;
    };
#endif
