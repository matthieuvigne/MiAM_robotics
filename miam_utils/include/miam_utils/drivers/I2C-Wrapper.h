/// \file drivers/I2C-Wrapper.h
/// \brief Wrapper for I2C communication.
///
/// \details This file implements helper functions to make I2C communication both easier and thread safe.
///             All these functions will write debug messages on the terminal on transfert failure.
///    \warning As several threads may require bus access at the same time, it is important that all I2C access
///             are done with these functions only.
///    \note     All functions in this header should be prefixed with i2c_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef I2C_WRAPPER
#define I2C_WRAPPER
    #include <string>
    #include <mutex>

    ///< Structure representing an I2C port access, with thread safety.
    typedef struct{
        int file;    ///< The file descriptor of the port number to use.
        std::mutex portMutex; ///< A mutex used internally to guarantee a thread-safe implementation.
    }I2CAdapter;

    /// \brief Open an I2C port.
    ///
    /// \param[in] adapter An I2CAdapter structure to fill.
    /// \param[in] portName Name of the I2C port to use (in the file system, i.e. a string "/dev/i2c-x").
    /// \returns true on success, false on failure.
    bool i2c_open(I2CAdapter *adapter, std::string const& portName);


    /// \brief Write data to an I2C register
    ///
    /// \param[in] adapter The I2CAdapter structure to use: this structure defines the port being used.
    /// \param[in] port File descriptor of the I2C port.
    /// \param[in] address Address of the device to talk to.
    /// \param[in] reg Address of the register to set.
    /// \param[in] data Data to write to the register.
    /// \returns true on success, false on failure.
    bool i2c_writeRegister(I2CAdapter *adapter, unsigned char const& address, unsigned char const& reg, unsigned char const& data);

    /// \brief Write several I2C registers at once.
    /// \note Not all device support writing several registers (autoincrement) - check device documentation beforehand.
    ///
    /// \param[in] adapter The I2CAdapter structure to use: this structure defines the port being used.
    /// \param[in] address Address of the device to talk to.
    /// \param[in] registerAddress Address of the first register to write.
    /// \param[in] length Number of registers to write.
    /// \param[in] values Char array of values to write.
    /// \returns true on success, false on failure.
    bool i2c_writeRegisters(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress, int const& length, unsigned char const *values);

    /// \brief Read an I2C register
    ///
    /// \param[in] adapter The I2CAdapter structure to use: this structure defines the port being used.
    /// \param[in] address Address of the device to talk to.
    /// \param[in] registerAddress Address of the register to read.
    /// \returns   The value of the specified register.
    unsigned char i2c_readRegister(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress);

    /// \brief Read several I2C registers at once.
    /// \note Not all device support reading several registers - check device documentation beforehand.
    /// \note This function performs no memory allocation.
    ///
    /// \param[in] adapter The I2CAdapter structure to use: this structure defines the port being used.
    /// \param[in] address Address of the device to talk to.
    /// \param[in] registerAddress Address of the first register to read.
    /// \param[in] length Number of registers to read.
    /// \param[out] output Char array of all the register read. Memory must have already been alocated.
    bool i2c_readRegisters(I2CAdapter *adapter, unsigned char const& address, unsigned char const& registerAddress, int const& length, unsigned char *output);

    /// \brief Close an I2C port.
    ///
    /// \param[in] port File descriptor of the I2C port.
    void i2c_close(int port);
#endif
