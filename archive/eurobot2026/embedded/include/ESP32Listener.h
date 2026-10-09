#ifndef ESP32LISTENER_H
    #define ESP32LISTENER_H

    #include <iostream>
    #include <miam_utils/COBSSerialListener.h>

    struct ESP32Data {
        int32_t obstacleDistance = 0;
        float angle = 0.0;
    };
    std::ostream& operator<<(std::ostream& os, ESP32Data const& d);

    /// @brief Listen to ESP32, communicating using COBS
    class ESP32Listener
    {
        public:
            /// \brief Constructor: do nothing for now.
            ESP32Listener();

            /// \brief Initialize listener
            ///
            /// \param portName Serial port file name ("/dev/ttyOx")
            /// \param baudRate Baud rate
            bool init(std::string const& portName, int const& baudRate = 1000000);

            /// @brief Call periodically to process serial port
            /// @return True if new data is available (use getLastData)
            bool update();

            /// Get last data from ESP32
            ESP32Data getLastData();

        private:
            CobsSerialListener listener_;
            ESP32Data lastData_;
    };
 #endif
