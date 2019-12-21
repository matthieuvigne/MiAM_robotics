/// \file drivers/MaestroServoDriver.h
/// \brief Driver for the maestro servo drivers, using UART.
///
/// \details For now only target setting via pololu protocol is supported.
///    \note     All functions in this header should be prefixed with maestro_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MAESTROSERVO_DRIVER
    #define MAESTROSERVO_DRIVER

    #include <string>

    /// \brief MaestroDriver Pololu servo driver, using USB or UART communication.
    /// \details The so-called "Pololu protocol" is used, without CRC check.
    class MaestroDriver
    {
        public:
            /// \brief Default contstructor.
            MaestroDriver();

            /// \brief Initialize the servo driver.
            ///
            /// \param portName Serial port file name ("/dev/ttyOx")
            /// \param deviceID Maestro device ID.
            /// \returns   true on success, false otherwise.
            bool init(std::string const& portName, int const& deviceID = 12);

            /// \brief Set target position of a servo.
            ///
            /// \param[in] servo The number of the servo to change (from 0 to 15).
            /// \param[in] position Signal value, in microseconds (clamped between 500 and 2500). Note that the resolution
            ///                   of the driver is of 0.25 microseconds.
            void setPosition(int const& servo, double const& position);

            /// \brief Set target speed of a servo.
            /// \details This function in itself does not move a servo, but only specify the speed at which it will move at.
            ///
            /// \param[in] servo The number of the servo to change (from 0 to 15).
            /// \param[in] speed Servo speed, in us/s. Device resolution is 25us/s
            void setSpeed(int const& servo, int const& speed);
        private:

            /// \brief Clear internal device error.
            void clearError();

            /// \brief Send a command to the driver.
            /// \param[in] commandID ID of the command.
            /// \param[in] parameters Command parameters.
            /// \param[in] length Length of the parameter.
            /// \return Result of write.
            int sendCommand(int const& commandID, unsigned char *parameters, int const& length);

            int port_;        ///< Serial port file descriptor.
            int deviceID_;     ///< Pololu device ID, for daisy chaining.
    };
#endif
