/// \file RMDX.h
/// \brief A simple driver for the RMDX brushless motor from MyActuator
///
/// \details This code is meant to work as a simple example for communicating with
///          the RMDX brushless motors. Note that only single-motor commands are implemented.

#ifndef MYACTUATOR_RMDX
#define MYACTUATOR_RMDX

#include "miam_utils/drivers/MCP2515Driver.h"
#include <iostream>
#include <mutex>


namespace MyActuator::commands
{
    unsigned char const READ_POS_KP = 0x30;
    unsigned char const READ_POS_KI = 0x31;
    unsigned char const READ_VEL_KP = 0x32;
    unsigned char const READ_VEL_KI = 0x33;
    unsigned char const READ_TORQUE_KP = 0x34;
    unsigned char const READ_TORQUE_KI = 0x35;
    unsigned char const WRITE_POS_KP_RAM = 0x36;
    unsigned char const WRITE_POS_KI_RAM = 0x37;
    unsigned char const WRITE_VEL_KP_RAM = 0x38;
    unsigned char const WRITE_VEL_KI_RAM = 0x39;
    unsigned char const WRITE_TORQUE_KP_RAM = 0x3A;
    unsigned char const WRITE_TORQUE_KI_RAM = 0x3B;
    unsigned char const WRITE_POS_KP_ROM = 0x3C;
    unsigned char const WRITE_POS_KI_ROM = 0x3D;
    unsigned char const WRITE_VEL_KP_ROM = 0x3E;
    unsigned char const WRITE_VEL_KI_ROM = 0x3F;
    unsigned char const WRITE_TORQUE_KP_ROM = 0x40;
    unsigned char const WRITE_TORQUE_KI_ROM = 0x41;
    unsigned char const READ_ACCEL = 0x42;
    unsigned char const WRITE_ACCEL = 0x43;
    unsigned char const READ_MULTITURN_POS = 0x60;
    unsigned char const READ_ORIGINAL_POS = 0x61;
    unsigned char const READ_MULTITURN_OFFSET = 0x62;
    unsigned char const WRITE_ENCODER_ZERO = 0x63;
    unsigned char const WRITE_ENCODER_CURRENT_POS_AS_ZERO = 0x64;
    unsigned char const READ_MULTITURN_ANGLE = 0x92;

    unsigned char const READ_MOTOR_STATUS1 = 0x9A;
    unsigned char const READ_MOTOR_STATUS2 = 0x9C;
    unsigned char const READ_MOTOR_STATUS3 = 0x9D;

    unsigned char const SHUTDOWN = 0x80;
    unsigned char const STOP = 0x81;
    unsigned char const ENABLE = 0x88;

    unsigned char const TORQUE_COMMAND = 0xA1;
    unsigned char const SPEED_COMMAND = 0xA2;
    unsigned char const ABS_POS_COMMAND = 0xA4;
    unsigned char const REL_POS_COMMAND = 0xA8;


    unsigned char const READ_OPERATING_MODE = 0x70;
    unsigned char const READ_MOTOR_POWER = 0x71;
    unsigned char const READ_AUXILIARY_VOLTAGE = 0x72;
    unsigned char const WRITE_TORQUE_FEEDFORWARD = 0x73;

    unsigned char const RESET = 0x76;
    unsigned char const BRAKE_RELEASE = 0x77;
    unsigned char const BRAKE_LOCK = 0x78;
    unsigned char const CAN_ID_SETUP = 0x79;
    unsigned char const READ_RUNTIME = 0xB1;
    unsigned char const READ_SOFTWARE_VERSION = 0xB2;
    unsigned char const COMM_INTERRUPT_TIMEOUT = 0xB3;
}



class RMDX{
    public:
        enum ErrorCode
        {
            OK,
            TIMEOUT,
            BAD_REPLY
        };

        struct Status{
            double batteryVoltage = 0; ///< Battery voltage, V
            int motorTemperature = 0; ///< Motor temperature, deg C
            bool isBrakeOn = false; ///< Brake status
            int16_t motorStatus = 0; ///< Motor status
        };

        RMDX(MCP2515 *canDriver, double const& timeout = 0.002);

        /// \brief Reset a given motor
        /// \param[in] motorId Motor id
        void reset(unsigned char const& motorId);

        /// @brief  \brief Init communication with motor, checking that it is present
        /// @param motorId Motor id
        /// @param motorTimeout Motor timeout before autostop, in s.
        /// @return True if motor answered back.
        bool init(unsigned char const& motorId, double const& motorTimeout = 0.100);

        void enable(unsigned char const& motorId);
        void disable(unsigned char const& motorId);
        void stop(unsigned char const& motorId);

        /// \brief Set target speed, joint side.
        /// \param[in] motorId Motor id
        /// \param[in] targetSpeed Target speed, joint side, in rad/s
        /// \param[in] reductionRatio Motor reduction ratio
        /// \note On RMDXV2, speed control behavior is determined by PD gains, but also
        ///       by hidden acceleration limit parameter, only accessible through the debugging
        ///       software (not CAN).
        /// \return Current speed, in rad/s. 0 on failure.
        double setSpeed(unsigned char const& motorId, double const& targetSpeed, double const& reductionRatio = 1.0);


        /// \brief Set target position, joint side.
        /// \param[in] motorId Motor id
        /// \param[in] targetPosition Target speed, joint side, in rad/
        /// \param[in] reductionRatio Motor reduction ratio
        /// \return Current position, in rad. 0 on failure.
        double setPosition(unsigned char const& motorId, double const& targetPosition, double const& reductionRatio = 1.0);

        /// \brief Set target speed, joint side.
        /// \param[in] motorId Motor id
        /// \param[in] targetCurrent Target current, in A
        /// \return Actual current
        double setCurrent(unsigned char const& motorId, double const& targetCurrent);

        /// \brief Get the current joint position, in rad.
        /// \param[in] motorId Motor id
        /// \param[in] reductionRatio Reduction ratio
        /// \return Joint angular position, in rad.
        double getCurrentPosition(unsigned char const& motorId, double const& reductionRatio = 1.0);

        /// \brief Get motor status
        /// \param[in] motorId Motor id
        /// \return Motor status (voltage, temperature, error code)
        RMDX::Status getStatus(unsigned char const& motorId);


        /// \brief Turn brake on or off
        /// \param[in] motorId Motor id
        /// \param[in] turnBrakeOn Desired brake status
        void setBrake(unsigned char const& motorId, bool const& turnBrakeOn);

        ErrorCode lastError_;
        std::mutex mutex_;
    private:

        /// \brief Set communication timeout (0 to disable)
        /// \param[in] motorId Motor id
        /// \param[in] timeoutMS Timeout, in ms.
        /// \return True on success
        bool setCommunicationTimeout(unsigned char const& motorId, int32_t const& timeoutMS);

        /// \brief Send and recieve a data frame from the motor.
        ///
        /// \param[in, out] message The message to send, modified in place.
        /// \param waitForReply Whether or not to wait for a reply
        /// \return 0 on succes ; -1 if write failed ; -2 if write succeeded but read failed.
        int canReadWrite(CANMessage& message, bool const& waitForReply = true);

        MCP2515 *canDriver_;
        double timeout_;
};

inline std::ostream& operator << (std::ostream& o, RMDX::Status const& s)
{
    o << "Motor temp: " << s.motorTemperature;
    o << " voltage: " << s.batteryVoltage;
    o << " brake: " << (s.isBrakeOn ? "on": "off");
    o << " status: " << s.motorStatus;
    return o;
}

#endif
