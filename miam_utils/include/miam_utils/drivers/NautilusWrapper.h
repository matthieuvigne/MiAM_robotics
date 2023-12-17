/// \file NautilusWrapper.h
/// \brief A wrapper for the Nautilus motor driver

#ifndef NAUTILUS_WRAPPER_H
    #define NAUTILUS_WRAPPER_H
    #include "miam_utils/drivers/NautilusDriver.h"

    struct NautilusMeasurements{
        double encoderPosition = 0.0; // rad
        double motorVelocity = 0.0; // rad/s
        double motorCurrent = 0.0; // A
        double batteryVoltage = 0.0; // V
        uint16_t currentMode = 0; // mode of operation
        int nCommunicationFailed = 0; // number of failed communication since start
    };

    class NautilusWrapper{
        public:
            /// \brief Constructor.
            NautilusWrapper(std::string const& portName, int const& frequency = 4000000);
            ~NautilusWrapper();

            /// @brief Initialize and check communication with driver.
            /// @param isEncoderInit State of the encoder
            /// @return True is driver is present and responds correctly.
            bool init(bool & isEncoderInit);

            NautilusMeasurements updateMeasurements();

            /// @brief Set motor target velocity
            /// @param targetVelocity Target velocity (rad/s).
            void setTargetVelocity(double const& targetVelocity);

            /// @brief Stop the motor, holding the position fixed.
            void stop();

            /// @brief Disable the motor, applying no torque.
            void disable();

        private:
            /// @brief Try to read a register, retrying up to N timesuntil it worked.
            /// @param reg Register to read
            /// @param nRetries Number of retries
            /// @return First valid reply
            nautilus::NautilusReply readRegister(nautilus::Register const& reg, int nRetries = 3);
            nautilus::Nautilus nautilus_;
            NautilusMeasurements lastMeasurements_;
            bool hadValidMeasurement_{false};
            bool isStoppedPositionValid_{false};
            float stoppedPosition_{0.0f};

            double oldEncoderPosition_{0.0};
    };
#endif

