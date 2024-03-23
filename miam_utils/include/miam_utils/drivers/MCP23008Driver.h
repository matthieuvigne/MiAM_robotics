#ifndef MPC23008_DRIVER
    #define MCP23008_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"

    /// @brief  \brief A simple driver for the MPC23008, used only as outputs
    class MCP23008{
        public:
            MCP23008();

            /// @brief Init and test communication with driver.
            /// @return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x20);

            /// @brief Set a single pin.
            /// @param[in] portId Id of the port to set (0-7)
            /// @param[in] high True to set to high (3.3V), false for low.
            void setPin(unsigned char const& portId, bool const& high);

            /// @brief  Set the state of all outputs.
            /// @param bitMask Bit mask of output state.
            void setOutputs(unsigned char const& bitMask);


        private:
            I2CAdapter *adapter_;       ///< I2C port file descriptor.
            unsigned char address_;    ///< Led driver address.

            bool isInit_;   ///< Status of initialization.

            unsigned char currentState_;
    };
#endif
