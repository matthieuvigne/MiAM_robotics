#ifndef COBS_SERIAL_LISTENER_H
    #define COBS_SERIAL_LISTENER_H

    #include <string>
    #include <vector>
    #include <termios.h>

    /// @brief Listen to incomming COBS packages from the serial port, in a non-blocking fashion
    class CobsSerialListener
    {
        public:
            CobsSerialListener();

            /// \brief Initialize listener
            ///
            /// \param portName Serial port file name ("/dev/ttyOx")
            /// \param baudRate Baud rate
            /// \returns  True on success
            bool init(std::string const& portName, int const& baudRate = B1000000);

            /// @brief Call periodically to flush serial data
            /// @return True if new COBS is available.
            bool update();

            /// @brief Return copy of last frame read.
            /// @return Last decoded COBS message.
            std::vector<uint8_t> getLastFrame();

        private:
            int port_ = -1; ///< Port to listen to for the ESP32

            std::vector<uint8_t> inputBuffer_;
            std::vector<uint8_t> outputBuffer_;
    };
 #endif
