/// \file Register.h
/// \brief Registers list for Nautilus

#ifndef NAUTILUS_REGISTERS_H
    #define NAUTILUS_REGISTERS_H
    #include <vector>

namespace nautilus
{

    enum class Register{
        currentMode             = 0x00,
        faultCode               = 0x01,
        drvStatus               = 0x02,
        drvConfigError          = 0x03,

        measuredPosition        = 0x10,
        measuredVelocity        = 0x11,
        measuredIQ              = 0x12,
        measuredIPhaseA         = 0x13,
        measuredIPhaseB         = 0x14,
        measuredIPhaseC         = 0x15,
        measuredUPhaseA         = 0x16,
        measuredUPhaseB         = 0x17,
        measuredUPhaseC         = 0x18,
        measuredMotTemp         = 0x19,
        measuredDriveTemp       = 0x1A,
        measuredUBat            = 0x1B,

        targetPosition          = 0x20,
        targetVelocity          = 0x21,
        targetIQ                = 0x22,

        rawEncoderPos           = 0x30,
        encoderOrientation      = 0x31,
        commutationOffset       = 0x32,
        nbrOfPoles              = 0x33,

        currentLoopKp           = 0x40,
        currentLoopKi           = 0x41,
        currentLoopIntMax       = 0x42,
        velocityLoopKp          = 0x43,
        velocityLoopKi          = 0x44,
        velocityLoopIntMax      = 0x45,

        positionLoopKp          = 0x46,
        positionLoopKd          = 0x47,
        positionLoopKi          = 0x48,
        positionLoopIntMax      = 0x49,

        motorMaxCurrent         = 0x50,
        motorMaxTemperature     = 0x51,
        driverMaxTemperature    = 0x52,
        commTimeout             = 0x53,
    };

    enum class Mode{
        Stopped     = 0,
        Fault       = 1,
        Enabling    = 2,
        Calibrating = 3,
        CalibrationComplete = 4,
        Pwm = 5,
        Voltage = 6,
        VoltageFoc = 7,
        VoltageDq = 8,
        Current = 9,
        Position = 10,
        PositionTimeout = 11,
        ZeroVelocity = 12,
        StayWithinBounds = 13,
        MeasureInductance = 14,
        Brake = 15,
        Velocity = 16,
    };

    // Extra information about the registers, for the GUI.
    struct GUIRegister{
        Register address;
        std::string name;
        bool isFloat;
        bool isWritable;

        GUIRegister(Register const& ad, std::string const& n, bool const& isF, bool const& isW):
            address(ad),
            name(n),
            isFloat(isF),
            isWritable(isW)
        {}
    };

    static std::vector<GUIRegister> registerList({
        GUIRegister(Register::currentMode, "Current mode",        false, false),
        GUIRegister(Register::faultCode, "Fault code",          false, false),
        GUIRegister(Register::drvStatus, "DRV status",          false, false),
        GUIRegister(Register::drvConfigError, "DRV config error",    false, false),

        GUIRegister(Register::measuredPosition, "Measured position",   true, false),
        GUIRegister(Register::measuredVelocity, "Measured velocity",   true, false),
        GUIRegister(Register::measuredIQ, "Measured Iq",         true, false),
        GUIRegister(Register::measuredIPhaseA, "Measured I phase A",  true, false),
        GUIRegister(Register::measuredIPhaseB, "Measured I phase B",  true, false),
        GUIRegister(Register::measuredIPhaseC, "Measured I phase c",  true, false),
        GUIRegister(Register::measuredUPhaseA, "Measured U phase A",  true, false),
        GUIRegister(Register::measuredUPhaseB, "Measured U phase B",  true, false),
        GUIRegister(Register::measuredUPhaseC, "Measured U phase C",  true, false),
        GUIRegister(Register::measuredMotTemp, "Measured mot temp",   true, false),
        GUIRegister(Register::measuredDriveTemp, "Measured fet temp",   true, false),
        GUIRegister(Register::measuredUBat, "Measured battery voltage",   true, false),

        GUIRegister(Register::targetPosition, "Target position", true, false),  // Can only be written using dedicated interface
        GUIRegister(Register::targetVelocity, "Target velocity", true, false),  // Can only be written using dedicated interface
        GUIRegister(Register::targetIQ, "Target Iq",       true, false),        // Can only be written using dedicated interface

        GUIRegister(Register::rawEncoderPos, "Raw position",        false, false),
        GUIRegister(Register::encoderOrientation, "Encoder orientation", false, true),
        GUIRegister(Register::commutationOffset, "Commutation offset",  true, true),
        GUIRegister(Register::nbrOfPoles, "Number of poles",     false, false),

        GUIRegister(Register::currentLoopKp, "Current loop Kp", true, true),
        GUIRegister(Register::currentLoopKi, "Current loop Ki", true, true),
        GUIRegister(Register::currentLoopIntMax, "Current loop max integral", true, true),
        GUIRegister(Register::velocityLoopKp, "Velocity loop Kp", true, true),
        GUIRegister(Register::velocityLoopKi, "Velocity loop Ki", true, true),
        GUIRegister(Register::velocityLoopIntMax, "Velocity loop max integral", true, true),
        GUIRegister(Register::positionLoopKp, "Position loop Kp", true, true),
        GUIRegister(Register::positionLoopKd, "Position loop Kd", true, true),
        GUIRegister(Register::positionLoopKi, "Position loop Ki", true, true),
        GUIRegister(Register::positionLoopIntMax, "Position loop max integral", true, true),
        GUIRegister(Register::motorMaxCurrent, "Motor max current", true, true),
        GUIRegister(Register::motorMaxTemperature, "Motor max temperature", true, true),
        GUIRegister(Register::driverMaxTemperature, "Driver max temperature", true, true),
        GUIRegister(Register::commTimeout, "Communication timeout ms", false, true),
    });



}
#endif

