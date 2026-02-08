/// \file drivers/STSScheduler.h
/// \brief Thread-based handling of sts servos
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef STSSERVO_MANAGER
    #define STSSERVO_MANAGER

    #include <thread>
    #include <vector>
    #include <memory>
    #include "miam_utils/drivers/STSServoDriver.h"
    #include "miam_utils/RailServo.h"

    /// \brief Thread managing the sts servos
    class STSScheduler
    {
        public:
            /// \brief Default contstructor.
            STSScheduler(double const& readTimeout = 0.000750);

            ~STSScheduler();

            /// @brief Shutdown all servos and exit background thread.
            void shutdown();

            /// \brief Initialize the servo driver.
            ///
            /// \param portName Serial port file name ("/dev/ttyOx")
            /// \param dirPort Direction port GPIO number.
            /// \param baudRate Baud rate
            /// \returns  True on success (at least one servo responds to ping)
            bool init(std::string const& portName, int const& dirPort, int const& baudRate = B1000000);

            /// @brief Create a rail servo
            /// @param servoId
            /// @param gpioId
            /// @param distance
            /// @param inverted
            /// @param calibrateBottom
            /// @return Pointer to created RailServo object
            std::shared_ptr<RailServo> createRail(int const& servoId, int const& gpioId, int const& distance, bool inverted=false, bool calibrateBottom=false);

            /// @brief Start calibration of the rails
            void startRailCalibration();

            bool areAllRailsCalibrated() const;

            /// \brief Set servo working mode: position, velocity or step.
            /// \param[in] servoId ID of the servo
            /// \param[in] mode Desired mode
            void setMode(unsigned char const& servoId, STS::Mode const& mode);

            /// \brief Get current servo position.
            /// \param[in] servoId ID of the servo
            /// \return Position, in deg. 0 on failure.
            int16_t getCurrentPosition(unsigned char const& servoId);

            /// \brief Get current servo speed.
            /// \param[in] servoId ID of the servo
            /// \return Speed, in ticks/s. 0 on failure.
            int16_t getCurrentVelocity(unsigned char const& servoId);

            /// \brief Check if the servo is moving
            /// \param[in] servoId ID of the servo
            /// \return True if moving, false otherwise.
            bool isMoving(unsigned char const& servoId);

            /// \brief Set target servo position.
            /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
            /// \param[in] servoId ID of the servo
            /// \param[in] position Target position, in ticks.
            /// \return True on success, false otherwise.
            void setTargetPosition(unsigned char const& servoId, int16_t const& position);

            /// \brief Set target servo velocity.
            /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
            /// \param[in] servoId ID of the servo
            /// \param[in] velocity Target velocity, in ticks/s.
            /// \return True on success, false otherwise.
            void setTargetVelocity(unsigned char const& servoId, int16_t const& velocity);

            /// \brief Enable / disable servo torque output.
            /// \param[in] servoId ID of the servo
            /// \param[in] enable True: disable servo ; False: enable.
            /// \return True on success, false otherwise.
            void disable(unsigned char const& servoId);
            void enable(unsigned char const& servoId);

            int getNReadFailed() {return driver_.getNReadFailed();}
        private:

            enum class State
            {
                UNMANAGED,
                ENABLING,
                ON,
                DISABLING
            };

            struct ServoCommand{
                State state = State::UNMANAGED;
                STS::Mode mode = STS::Mode::POSITION;
                int16_t value = 0;
                int16_t resetCounter_ = 10;
            };

            ServoCommand commands_[256];
            std::vector<std::shared_ptr<RailServo>> rails_;
            std::mutex mutex_;


            void backgroundThread();
            bool askedForShutdown_ = false;
            std::thread bgThread_;
            STSServoDriver driver_;
    };
#endif
