#include "miam_utils/drivers/NautilusWrapper.h"
#include <cmath>

double const ENC_TICK_TO_RAD = 2 * M_PI / 16384.0;

using namespace nautilus;

NautilusWrapper::NautilusWrapper(std::string const& portName, double const& reductionRatio, int const& frequency):
    nautilus_(portName, frequency),
    motorReductionRatio_(reductionRatio)
{
}


NautilusWrapper::~NautilusWrapper()
{

}


bool NautilusWrapper::init(bool & isEncoderInit)
{
    // Note: enabeling nautilus disables the encoder, so we check the
    // status only after having enabled the motor.
    disable();
    lastMeasurements_.currentMode = 0;
    stop();
    usleep(15000);
    NautilusReply rep = readRegister(Register::measuredVelocity);
    isEncoderInit = rep.isValid && rep.isEncoderValid;
    isInit_ = rep.isValid & isEncoderInit;
    return rep.isValid && rep.mode == static_cast<uint16_t>(Mode::Position);
}


NautilusMeasurements NautilusWrapper::updateMeasurements()
{
    lastMeasurements_.isEncoderPositionValid = false;
    NautilusReply rep = readRegister(Register::measuredVelocity);
    if (rep.isValid)
        lastMeasurements_.motorVelocity = rep.data / motorReductionRatio_;

    rep = readRegister(Register::measuredPosition);
    if (rep.isValid)
        lastMeasurements_.motorPosition = rep.data / motorReductionRatio_;

    rep = readRegister(Register::measuredIQ);
    if (rep.isValid)
        lastMeasurements_.motorCurrent = rep.data;

    rep = readRegister(Register::measuredUBat);
    if (rep.isValid)
        lastMeasurements_.batteryVoltage = rep.data;

    lastMeasurements_.nCommunicationFailed = nautilus_.nFailed;
    return lastMeasurements_;
}


void NautilusWrapper::setTargetVelocity(double const& targetVelocity)
{
    // Reset on error
    if (lastMeasurements_.currentMode == static_cast<uint16_t>(Mode::Fault))
    {
        nautilus_.stop();
    }
    else if (lastMeasurements_.currentMode != static_cast<uint16_t>(Mode::Velocity))
    {
        // Give it some time, then reset.
        nConsecutiveNoVel_ ++;
        if (nConsecutiveNoVel_ > 5)
        {
            nautilus_.stop();
        }
    }
    nautilus_.writeRegister(Register::targetVelocity, static_cast<float>(motorReductionRatio_ * targetVelocity));
}


void NautilusWrapper::stop()
{
    if (lastMeasurements_.currentMode != static_cast<uint16_t>(Mode::Position))
    {
        NautilusReply rep = readRegister(Register::measuredPosition);
        if (rep.isValid)
            stoppedPosition_ = rep.data;
        isStoppedPositionValid_ = rep.isValid;
    }
    if (isStoppedPositionValid_)
        nautilus_.writeRegister(Register::targetPosition, stoppedPosition_);
    nConsecutiveNoVel_ = 0;
}


void NautilusWrapper::disable()
{
    nautilus_.stop();
    nConsecutiveNoVel_ = 0;
}


NautilusReply NautilusWrapper::readRegister(nautilus::Register const& reg, int nRetries)
{
    NautilusReply rep;
    int i = 0;
    while (!rep.isValid && !rep.isEncoderValid && i < nRetries)
    {
        rep = nautilus_.readRegister(reg);
        i++;
    }
    if (isInit_ && rep.isValid && !rep.isEncoderValid)
        nEncoderInvalid_ ++;

    if (rep.isValid)
    {
        lastMeasurements_.currentMode = rep.mode;
        if (!lastMeasurements_.isEncoderPositionValid && rep.isEncoderValid)
        {
            lastMeasurements_.isEncoderPositionValid = true;

            double const encoderPosition = rep.encoderPosition * ENC_TICK_TO_RAD;
            double increment = encoderPosition - oldEncoderPosition_;
            if (increment > M_PI)
                increment = increment - 2 * M_PI;
            if (increment < -M_PI)
                increment = increment + 2 * M_PI;
            oldEncoderPosition_ = encoderPosition;
            lastMeasurements_.encoderPosition += increment;
        }
    }
    return rep;
}

std::string NautilusWrapper::getDebugStatus()
{
    NautilusReply rep = nautilus_.readRegister(nautilus::Register::faultCode);
    std::string output = "current mode: " + std::to_string(static_cast<int>(rep.mode));
    output += " fault code: " + std::to_string(reinterpret_cast<uint32_t &>(rep.data));
    rep = nautilus_.readRegister(nautilus::Register::drvStatus);
    output += " DRV status: " + std::to_string(reinterpret_cast<uint32_t &>(rep.data));
    return output;
}