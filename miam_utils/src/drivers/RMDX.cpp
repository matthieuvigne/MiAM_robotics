#include "miam_utils/drivers/RMDX.h"
#include "miam_utils/TextLogger.h"

#include <math.h>
#include <unistd.h>
#include <iostream>

double const RAD_TO_DEG = 180 / M_PI;

// Utility functions
CANMessage createMessage(unsigned char const& motorId, unsigned char const& command, int32_t const& data = 0)
{
    CANMessage message;

    message.id = 0x140 + motorId;
    message.len = 8;
    message.data[0] = command;
    message.data[4] = data & 0xFF;
    message.data[5] = (data >> 8) & 0xFF;
    message.data[6] = (data >> 16) & 0xFF;
    message.data[7] = (data >> 24) & 0xFF;
    return message;
}


int32_t messageToInt32(CANMessage const& message)
{
    return message.data[4] + (message.data[5] << 8) +  (message.data[6] << 16) +  (message.data[7] << 24);
}

uint32_t messageToUInt32(CANMessage const& message)
{
    return message.data[4] + (message.data[5] << 8) +  (message.data[6] << 16) +  (message.data[7] << 24);
}

RMDX::RMDX(MCP2515 *canDriver, double const& timeout):
    canDriver_(canDriver),
    timeout_(timeout)
{

}

bool RMDX::init(unsigned char const& motorId, double const& motorTimeout)
{
    // Test communication
    if (getStatus(motorId).batteryVoltage < 10)
        return false;

    // Configure motor
    enable(motorId);
    setCommunicationTimeout(motorId, static_cast<int32_t>(1000 * motorTimeout));

    return true;
}


void RMDX::reset(unsigned char const& motorId)
{
    CANMessage message = createMessage(motorId,  MyActuator::commands::RESET);
    canReadWrite(message, false);
}

void RMDX::enable(unsigned char const& motorId)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::ENABLE);
    canReadWrite(message, false);
}

void RMDX::stop(unsigned char const& motorId)
{
    // Hack: send 0 position increment to freeze the motor.
    CANMessage message = createMessage(motorId, MyActuator::commands::REL_POS_COMMAND);
    canReadWrite(message);
}

void RMDX::disable(unsigned char const& motorId)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::SHUTDOWN);
    canReadWrite(message, false);
}


double RMDX::setSpeed(unsigned char const& motorId, double const& targetSpeed, double const& reductionRatio)
{
    int32_t speedCount = static_cast<int32_t>(targetSpeed * RAD_TO_DEG * reductionRatio * 100);
    CANMessage message = createMessage(motorId, MyActuator::commands::SPEED_COMMAND, speedCount);
    if (canReadWrite(message) >= 0)
    {
        int16_t data = message.data[4] + (message.data[5] << 8);
        return static_cast<double>(data) / RAD_TO_DEG / reductionRatio;
    }
    // If no reply could be obtained, return target speed.
    return targetSpeed;
}

double RMDX::setPosition(unsigned char const& motorId, double const& targetPosition, double const& reductionRatio)
{
    int32_t positionCount = static_cast<int32_t>(targetPosition * RAD_TO_DEG * reductionRatio * 100);
    CANMessage message = createMessage(motorId, MyActuator::commands::ABS_POS_COMMAND, positionCount);
    if (canReadWrite(message) >= 0)
    {
        int16_t data = message.data[4] + (message.data[5] << 8);
        return static_cast<double>(data) / RAD_TO_DEG / reductionRatio;
    }
    // If no reply could be obtained, return target speed.
    return targetPosition;
}

double RMDX::setCurrent(unsigned char const& motorId, double const& targetCurrent)
{

    int16_t currentCount = static_cast<int16_t>(targetCurrent * 2000 / 32);

    CANMessage message = createMessage(motorId, MyActuator::commands::TORQUE_COMMAND);
    message.data[4] = currentCount & 0xFF;
    message.data[5] = (currentCount >> 8) & 0xFF;

    if (canReadWrite(message) > 0)
    {
        int16_t data = message.data[2] + (message.data[3] << 8);
        return static_cast<double>(data) * 33.0 / 2048.0;
    }
    // If no reply could be obtained, return target current.
    return targetCurrent;
}

double RMDX::getCurrentPosition(unsigned char const& motorId, double const& reductionRatio)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::READ_MULTITURN_ANGLE);
    if (canReadWrite(message) >= 0)
    {
        double const result = messageToInt32(message);
        return result / RAD_TO_DEG / 100.0 / reductionRatio;
    }
    return 0;
}


RMDX::Status RMDX::getStatus(unsigned char const& motorId)
{
    RMDX::Status status;
    CANMessage message = createMessage(motorId, MyActuator::commands::READ_MOTOR_STATUS1);
    if (canReadWrite(message) > 0)
    {
        status.motorTemperature = message.data[1];
        status.isBrakeOn = message.data[3] == 0;
        status.batteryVoltage = (message.data[4] + (message.data[5] << 8))  / 10.0;
        status.motorStatus = message.data[6] + (message.data[7] << 8);
    }
    return status;
}

bool RMDX::setCommunicationTimeout(unsigned char const& motorId, int32_t const& timeoutMS)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::COMM_INTERRUPT_TIMEOUT, timeoutMS);
    return canReadWrite(message, false) == 0;
}


int RMDX::canReadWrite(CANMessage& message, bool const& waitForReply)
{
    lastError_ = ErrorCode::OK;
    mutex_.lock();
    canDriver_->sendMessage(message);
    mutex_.unlock();

    // Clear pending message - the motor takes a lot of time to reply anyway.
    usleep(100);
    CANMessage dump;
    while (canDriver_->isDataAvailable())
        canDriver_->readAvailableMessage(dump);

    if (waitForReply)
    {
        struct timespec startTime, currentTime;
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        while (!canDriver_->isDataAvailable())
        {
            usleep(20);
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            double const elapsed = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1e9;
            if (elapsed > timeout_)
                break;
        }

        if (canDriver_->isDataAvailable())
        {
            unsigned char const originalCode = message.data[0];
            int const originalId = static_cast<int>(message.id - 0x140);
            canDriver_->readAvailableMessage(message);
            if (message.data[0] != originalCode)
            {
                texterror << "[RMDX] Error: motor " << originalId << ", unexpected reply to command" << static_cast<int>(originalCode) << ": " << static_cast<int>(message.data[0]) << std::endl;
                lastError_ = ErrorCode::BAD_REPLY;
                return -1;
            }
            return 1;
        }
        else
        {
            lastError_ = ErrorCode::TIMEOUT;
            texterror << "[RMDX] Error: motor " << static_cast<int>(message.id - 0x140) << ": timeout on message listening" << std::endl;
            return -2;
        }
    }
    return 0;
}


void RMDX::setBrake(unsigned char const& motorId, bool const& turnBrakeOn)
{
    CANMessage message;
    if (turnBrakeOn)
        message = createMessage(motorId, MyActuator::commands::BRAKE_LOCK);
    else
        message = createMessage(motorId, MyActuator::commands::BRAKE_RELEASE);
    canReadWrite(message, true);
}