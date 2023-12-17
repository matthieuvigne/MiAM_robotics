/// \file NautilusDriver.h
/// \brief Linux driver for Nautilus (to be used for instance on Raspberry Pi)

#ifndef NAUTILUS_DRIVER_H
    #define NAUTILUS_DRIVER_H
    #include <unistd.h>
    #include <linux/spi/spidev.h>
    #include <string>
    #include <mutex>

    #include "NautilusRegisters.h"

namespace nautilus
{

    struct NautilusReply{
        uint16_t encoderPosition = 0;
        bool isEncoderValid = false;
        uint8_t mode = 0;
        float data;
        bool isValid = false;
    };

    enum Command {
        regWrite = 0x01,
        regRead  = 0x02,
        commutation  = 0x03,
        storeToPersistentMemory  = 0x04,
        stop  = 0x05,
    };

    class Nautilus{
        public:
            /// \brief Constructor.
            Nautilus(std::string const& portName, int const& frequency = 4000000);
            ~Nautilus();

            /// \brief Read a register
            NautilusReply readRegister(Register const& reg);

            bool writeRegister(Register const& reg, float const& value);
            bool writeRegister(Register const& reg, uint32_t const& value);

            void storeToPersistentMemory();

            /// @brief Stop the motors
            NautilusReply stop();

            /// @brief Commutation command
            /// @param theta Angle, range 0-2 pi
            /// @param voltageRatio Ratio of max voltage, range 0-1
            NautilusReply commutation(float const& theta, float const& voltageRatio);

            /// @brief Test function: send an invalid command.
            void sendInvalidCommand();

            int nSuccess = 0;
            int nFailed = 0;
        private:
            /// @brief Perform SPI communication
            /// @param buffer Data buffer, will be overwritten. Must contain 8 elements
            /// @return Formatted reply from drive.
            NautilusReply spiComm(uint8_t *buffer);

            /// @brief Helper for SPI communication, constructing the message.
            /// @param command
            /// @param address
            /// @param data
            /// @return
            NautilusReply spiComm(uint8_t command, uint8_t address, uint32_t data);

            std::string portName_;  ///< Name of the SPI port
            int frequency_; ///< Frequency, in Hz.
            std::mutex mutex_;

            int fileDescriptor_;
    };
}
#endif

