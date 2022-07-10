/// \file drivers/STSServoServoDriver.h
/// \brief Driver for the STS-serie UART servo by Feetech
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef STSSERVO_DRIVER
    #define STSSERVO_DRIVER

    #include <string>
    #include <vector>
    #include <termios.h>

    namespace STS
    {
        namespace registers
        {
            unsigned char const FIRMWARE_MAJOR          = 0x00;
            unsigned char const FIRMWARE_MINOR          = 0x01;
            unsigned char const SERVO_MAJOR             = 0x03;
            unsigned char const SERVO_MINOR             = 0x04;
            unsigned char const ID                      = 0x05;
            unsigned char const BAUDRATE                = 0x06;
            unsigned char const RESPONSE_DELAY          = 0x07;
            unsigned char const RESPONSE_STATUS_LEVEL   = 0x08;
            unsigned char const MINIMUM_ANGLE           = 0x09;
            unsigned char const MAXIMUM_ANGLE           = 0x0B;
            unsigned char const MAXIMUM_TEMPERATURE     = 0x0D;
            unsigned char const MAXIMUM_VOLTAGE         = 0x0E;
            unsigned char const MINIMUM_VOLTAGE         = 0x0F;
            unsigned char const MAXIMUM_TORQUE          = 0x10;
            unsigned char const UNLOADING_CONDITION     = 0x13;
            unsigned char const LED_ALARM_CONDITION     = 0x14;
            unsigned char const POS_PROPORTIONAL_GAIN   = 0x15;
            unsigned char const POS_DERIVATIVE_GAIN     = 0x16;
            unsigned char const POS_INTEGRAL_GAIN       = 0x17;
            unsigned char const MINIMUM_STARTUP_FORCE   = 0x18;
            unsigned char const CK_INSENSITIVE_AREA     = 0x1A;
            unsigned char const CCK_INSENSITIVE_AREA    = 0x1B;
            unsigned char const CURRENT_PROTECTION_TH   = 0x1C;
            unsigned char const ANGULAR_RESOLUTION      = 0x1E;
            unsigned char const POSITION_CORRECTION     = 0x1F; //????
            unsigned char const OPERATION_MODE          = 0x21;
            unsigned char const TORQUE_PROTECTION_TH    = 0x22;
            unsigned char const TORQUE_PROTECTION_TIME  = 0x23;
            unsigned char const OVERLOAD_TORQUE         = 0x24;
            unsigned char const SPEED_PROPORTIONAL_GAIN = 0x25;
            unsigned char const OVERCURRENT_TIME        = 0x26;
            unsigned char const SPEED_INTEGRAL_GAIN     = 0x27;
            unsigned char const TORQUE_SWITCH           = 0x28;
            unsigned char const TARGET_ACCELERATION     = 0x29;
            unsigned char const TARGET_POSITION         = 0x2A;
            unsigned char const RUNNING_TIME            = 0x2C; //????
            unsigned char const RUNNING_SPEED           = 0x2E;
            unsigned char const TORQUE_LIMIT            = 0x30;
            unsigned char const WRITE_LOCK              = 0x37;
            unsigned char const CURRENT_POSITION        = 0x38;
            unsigned char const CURRENT_SPEED           = 0x3A;
            unsigned char const CURRENT_DRIVE_VOLTAGE   = 0x3C;
            unsigned char const CURRENT_VOLTAGE         = 0x3E;
            unsigned char const CURRENT_TEMPERATURE     = 0x3F;
            unsigned char const ASYNCHRONOUS_WRITE_ST   = 0x40;
            unsigned char const STATUS                  = 0x41;
            unsigned char const MOVING_STATUS           = 0x42;
            unsigned char const CURRENT_CURRENT         = 0x45;
        };
        enum class Mode : unsigned char
        {
            POSITION = 0,
            VELOCITY = 1,
            STEP = 3
        };
    };


    /// \brief Driver for STS servos, using UART
    class STSServoDriver
    {
        public:
            /// \brief Default contstructor.
            STSServoDriver(double const& readTimeout = 0.0015);

            /// \brief Initialize the servo driver.
            ///
            /// \param portName Serial port file name ("/dev/ttyOx")
            /// \param dirPort Direction port GPIO number.
            /// \param baudRate Baud rate
            /// \returns  True on success (at least one servo responds to ping)
            bool init(std::string const& portName, int const& dirPort, int const& baudRate = B1000000);

            /// \brief Ping servo
            /// \param[in] servoId ID of the servo
            /// \return True if servo responded to ping
            bool ping(unsigned char const& servoId);

            /// \brief Change the ID of a servo.
            /// \note If the desired ID is already taken, this function does nothing and returns false.
            /// \param[in] oldServoId old servo ID
            /// \param[in] newServoId new servo ID
            /// \return True if servo could successfully change ID
            bool setId(unsigned char const& oldServoId, unsigned char const& newServoId);

            /// \brief Test all addresses ans returns the list of servo that responded.
            /// \return std::vector of id who gave a response.
            std::vector<unsigned char> detectServos();

            /// \brief Set servo working mode: position, velocity or step.
            /// \param[in] servoId ID of the servo
            /// \param[in] mode Desired mode
            bool setMode(unsigned char const& servoId, STS::Mode const& mode);

            /// \brief Get current servo position.
            /// \param[in] servoId ID of the servo
            /// \return Position, in deg. 0 on failure.
            int16_t getCurrentPosition(unsigned char const& servoId);

            /// \brief Get current servo speed.
            /// \param[in] servoId ID of the servo
            /// \return Speed, in ticks/s. 0 on failure.
            int16_t getCurrentSpeed(unsigned char const& servoId);

            /// \brief Get current servo temperature.
            /// \param[in] servoId ID of the servo
            /// \return Temperature, in degC. 0 on failure.
            double getCurrentTemperature(unsigned char const& servoId);

            /// \brief Get current servo current.
            /// \param[in] servoId ID of the servo
            /// \return Current, in A.
            double getCurrentCurrent(unsigned char const& servoId);

            /// \brief Check if the servo is moving
            /// \param[in] servoId ID of the servo
            /// \return True if moving, false otherwise.
            bool isMoving(unsigned char const& servoId);

            /// \brief Set target servo position.
            /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
            /// \param[in] servoId ID of the servo
            /// \param[in] position Target position, in ticks.
            /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
            /// \return True on success, false otherwise.
            bool setTargetPosition(unsigned char const& servoId, int16_t const& position, bool const& asynchronous = false);

            /// \brief Set target servo velocity.
            /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
            /// \param[in] servoId ID of the servo
            /// \param[in] velocity Target velocity, in ticks/s.
            /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
            /// \return True on success, false otherwise.
            bool setTargetVelocity(unsigned char const& servoId, int16_t const& velocity, bool const& asynchronous = false);

            /// \brief Enable / disable servo torque output.
            /// \param[in] servoId ID of the servo
            /// \param[in] enable True: disable servo ; False: enable.
            /// \return True on success, false otherwise.
            bool disable(unsigned char const& servoId, bool const& disable = true);
            bool enable(unsigned char const& servoId) {return disable(servoId, false);};

            /// \brief Set the current position as the center of the servo range (2048).
            /// \param[in] servoId ID of the servo
            bool resetPositionAsCenter(unsigned char const& servoId);

            /// \brief Trigger the action previously stored by an asynchronous write on all servos.
            /// \return True on success
            bool trigerAction();

            /// \brief Write to a single byte register.
            /// \param[in] servoId ID of the servo
            /// \param[in] registerId Register id.
            /// \param[in] value Register value.
            /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
            /// \return True if write was successful
            bool writeRegister(unsigned char const& servoId,
                               unsigned char const& registerId,
                               unsigned char const& value,
                               bool const& asynchronous = false);

            /// \brief Write a two-bytes register.
            /// \param[in] servoId ID of the servo
            /// \param[in] registerId Register id (LSB).
            /// \param[in] value Register value.
            /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
            /// \return True if write was successful
            bool writeTwoBytesRegister(unsigned char const& servoId,
                                       unsigned char const& registerId,
                                       uint16_t const& value,
                                       bool const& asynchronous = false);

            /// \brief Read a single register
            /// \param[in] servoId ID of the servo
            /// \param[in] registerId Register id.
            /// \return Register value, 0 on failure.
            unsigned char readRegister(unsigned char const& servoId, unsigned char const& registerId);

            /// \brief Read two bytes, interpret result as <LSB> <MSB>
            /// \param[in] servoId ID of the servo
            /// \param[in] registerId LSB register id.
            /// \return Register value, 0 on failure.
            int16_t readTwoBytesRegister(unsigned char const& servoId, unsigned char const& registerId);


        private:

            /// \brief Clear internal device error.
            // void clearError();

            /// \brief Send a message to the servos.
            /// \param[in] servoId ID of the servo
            /// \param[in] commandID Command id
            /// \param[in] paramLength length of the parameters
            /// \param[in] parameters parameters
            /// \return Result of write.
            int sendMessage(unsigned char const& servoId,
                            unsigned char const& commandID,
                            unsigned char const& paramLength,
                            unsigned char *parameters);

            /// \brief Recieve a message from a given servo.
            /// \param[in] servoId ID of the servo
            /// \param[in] readLength Message length
            /// \param[in] paramLength length of the parameters
            /// \param[in] outputBuffer Buffer where the data is placed.
            /// \return 0 on success
            ///         -1 if read failed due to timeout
            ///         -2 if invalid message (no 0XFF, wrong servo id)
            ///         -3 if invalid checksum
            int recieveMessage(unsigned char const& servoId,
                               unsigned char const& readLength,
                               unsigned char *outputBuffer);

            /// \brief Write to a sequence of consecutive registers
            /// \param[in] servoId ID of the servo
            /// \param[in] startRegister First register
            /// \param[in] writeLength Number of registers to write
            /// \param[in] parameters Value of the registers
            /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
            /// \return True if write was successful
            bool writeRegisters(unsigned char const& servoId,
                                unsigned char const& startRegister,
                                unsigned char const& writeLength,
                                unsigned char const *parameters,
                                bool const& asynchronous = false);

            /// \brief Read a sequence of consecutive registers.
            /// \param[in] servoId ID of the servo
            /// \param[in] startRegister First register
            /// \param[in] readLength Number of registers to write
            /// \param[out] outputBuffer Buffer where to read the data (must have been allocated by the user)
            /// \return 0 on success, -1 if write failed, -2 if read failed, -3 if checksum verification failed
            int readRegisters(unsigned char const& servoId,
                              unsigned char const& startRegister,
                              unsigned char const& readLength,
                              unsigned char *outputBuffer);

            int port_;        ///< Serial port file descriptor.
            int dirPin_;     ///< Direction pin number.
            double readTimeout_; ///< Read timeout, in ms.
    };
#endif
