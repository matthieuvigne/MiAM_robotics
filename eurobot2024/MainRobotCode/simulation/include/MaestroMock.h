/// \file drivers/MaestroMock.h
/// \brief Mock of the Maestro driver for display
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MAESTRO_MOCK
    #define MAESTRO_MOCK

    #include <miam_utils/drivers/MaestroServoDriver.h>
    #include <string>
    #include <vector>

    /// \brief MaestroMock Mock of the maestro driver
    class MaestroMock: public MaestroDriver
    {
        public:
            /// \brief Default contstructor.
            MaestroMock();

            bool init(std::string const& portName, int const& deviceID = 12) override;

            /// \brief Set target position of a servo.
            ///
            /// \param[in] servo The number of the servo to change (from 0 to 15).
            /// \param[in] position Signal value, in microseconds (clamped between 500 and 2500). Note that the resolution
            ///                   of the driver is of 0.25 microseconds.
            void setPosition(int const& servo, double const& position) override;

            /// \brief Set target speed of a servo.
            /// \details This function in itself does not move a servo, but only specify the speed at which it will move at.
            ///
            /// \param[in] servo The number of the servo to change (from 0 to 15).
            /// \param[in] speed Servo speed, in us/s. Device resolution is 25us/s
            void setSpeed(int const& servo, int const& speed) override;

            std::vector<double> getState();
            void setState(std::vector<double> const& vectorIn);
        private:
            std::vector<double> state_;

    };
#endif
