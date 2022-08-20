/// \file drivers/MCP2515Driver.h
/// \brief Basic driver for the MCP2515 SPI-to-CAN interface.
///
/// \details This is a very basic driver for CAN bus using MCP2515 over the SPI interface:
///          only standard message without filtering is supported.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MPC2515_DRIVER
#define MPC2515_DRIVER

    #include <stdint.h>
    #include <string>
    #include <miam_utils/drivers/SPI-Wrapper.h>

    ///< A simple can message: only standard data frame.
    struct CANMessage{
        uint32_t id = 0 ;  // Frame identifier
        uint8_t len = 0 ;  // Length of data (0 ... 8)
        uint8_t  data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Actual data
    } ;


namespace miam{

    class MCP2515{
        public:
            /// \brief Default constructor.
            MCP2515();

            /// \brief Constructor.
            /// \details This function only builds the object, but does not perform any operation on the
            ///          SPI port.
            /// \param[in] spiDriver SPI driver
            MCP2515(SPIWrapper *spiDriver);

            /// \brief Try to init communication with the chip.
            ///
            /// \details This function tries, for each device, to reset it, then set its speed and velocity
            ///          profile, and its back-emf force. It then asks the device for a specific value, to
            ///          check the connection.
            ///
            /// \param[in] maxSpeed Maximum motor speed, in steps/s (from 15.25 to 15610, resolution 15.25 step/s).
            /// \return true is all devices responded, false otherwise.
            bool init();

            /// \brief Set a parameter register.
            ///
            ///
            /// \param[in] param The parameter to set.
            /// \param[out] paramValues Values of the parameter for each device.
            bool sendMessage(CANMessage const& message);

            bool isDataAvailable();

            bool readAvailableMessage(CANMessage & message);

        private:
            SPIWrapper *spiDriver_; ///< SPI port driver

            // Write to a given register.
            bool writeRegister(uint8_t const& address, uint8_t const& data);
            uint8_t readRegister(uint8_t const& address);


    };
}
#endif
