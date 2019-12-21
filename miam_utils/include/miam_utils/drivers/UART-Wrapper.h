/// \file drivers/UART-Wrapper.h
/// \brief Helper for using UART serial port.
///
/// \details This helper enables easy opening of a UART port.
///    \note     All functions in this header should be prefixed with uart_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef UART_WRAPPER
    #define UART_WRAPPER

    #include <termios.h>
    #include <string>

    /// \brief Open a serial communication for the given file name, at the given speed.
    ///
    /// \param portName Serial port file name ("/dev/ttyOx")
    /// \param speed Communication speed (i.e. B115200, or one of the constants defined in termios.h
    /// \return The open port file descriptor (positive int), or -1 on failure.
    int uart_open(std::string const& portName, int speed);

    /// \brief Wrapper around read function with a timeout.
    ///
    /// \details This function attempts to read at most size bytes from a given file descriptor, with
    ///          a timeout. If no data is read, it returns -1, otherwise this function behaves the same
    ///          as the read function.
    ///
    /// \param[in] file File descriptor.
    /// \param[out] buffer Buffer to fill - must be preallocated.
    /// \param[in] size Maximum amount of data to read - buffer must be at least size long.
    /// \param[in] timeoutMs Timeout in milliseconds.
    ///
    /// \return -1 on timeout, or the same as the underlying read call
    int read_timeout(int const& file, unsigned char *buffer, size_t const& size, uint const& timeoutMs);
#endif
