/// \file drivers/SPI-Wrapper.h
/// \brief Thread-safe wrapper for SPI communication.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SPI_WRAPPER
#define SPI_WRAPPER

    #include <string>
    #include <mutex>

    /// \brief Thread-safe wrapper for SPI communication,.
    class SPIWrapper{
        public:
            /// \brief Constructor
            /// \param[in] portName Name of the port
            /// \param[in] frequency Clock frequency
            SPIWrapper(std::string const& portName, int const& frequency = 4000000);

            /// \brief Send and receives an array of data over spi.
            /// \param[in] data Data to send - the buffer is overwritten with new data.
            /// \param[in] len Length of the input buffer.
            /// \return <0 on error.
            int spiReadWriteSingle(uint8_t* data, uint8_t const& len);

            int spiReadWrite(uint8_t const& numberOfPackets, struct spi_ioc_transfer* spiCtrl);

        private:
            // Start communication
            void spi_open();
            // End communication
            void spi_close();

            std::string portName_;  ///< Name of the SPI port
            int fd_; ///< File descriptor of the open port
            int frequency_; ///< Frequency, in Hz.
            std::recursive_mutex mutex_;    ///< Mutex, for thread safety. recursive_mutex that can be locked several
                                            /// times by the same thread: is this to prevent deadlock when sending a kill signal to the code.

    };

#endif
