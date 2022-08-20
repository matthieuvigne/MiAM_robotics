#include "miam_utils/drivers/RMDX.h"

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
    return message.data[4] + (message.data[5] >> 8) +  (message.data[6] >> 16) +  (message.data[7] >> 24);
}

namespace miam
{

      RMDX::RMDX(MCP2515 *canDriver, double const& timeout):
    canDriver_(canDriver),
    timeout_(timeout)
{

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
    CANMessage message = createMessage(motorId, MyActuator::commands::STOP);
    canReadWrite(message, false);
}

void RMDX::disable(unsigned char const& motorId)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::SHUTDOWN);
    canReadWrite(message, false);
}


double RMDX::getMaxAcceleration(unsigned char const& motorId, double const& reductionRatio)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::READ_ACCEL);
    if (canReadWrite(message) > 0)
        return messageToInt32(message) / RAD_TO_DEG / reductionRatio;

    return -1;
}

bool RMDX::setMaxAcceleration(unsigned char const& motorId, double const& targetAcceleration, double const& reductionRatio)
{
    int32_t accelCount = static_cast<int32_t>(targetAcceleration * RAD_TO_DEG * reductionRatio);
    CANMessage message = createMessage(motorId, MyActuator::commands::WRITE_ACCEL, accelCount);
    return canReadWrite(message, false) >= 0;
}

double RMDX::setSpeed(unsigned char const& motorId, double const& targetSpeed, double const& reductionRatio)
{
    int32_t speedCount = static_cast<int32_t>(targetSpeed * RAD_TO_DEG * reductionRatio * 100);
    CANMessage message = createMessage(motorId, MyActuator::commands::SPEED_COMMAND, speedCount);
    int result = canReadWrite(message);
    if (result <= 0)
        return 0;
    return static_cast<double>(message.data[4] + (message.data[5] << 8)) / RAD_TO_DEG / reductionRatio;
}


double RMDX::getCurrentPosition(unsigned char const& motorId, double const& reductionRatio)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::READ_MULTITURN_ANGLE);
    if (canReadWrite(message) > 0)
    {
        double result = 0;
        for (int i = 0; i < 6; i++)
            result += message.data[1 + i] << (8 * i);
        if (message.data[7] != 0)
            result = -result;
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

int RMDX::getMode(unsigned char const& motorId)
{
    CANMessage message = createMessage(motorId, 0x70);
    if (canReadWrite(message) > 0)
        return message.data[7];
    return 0;
}

bool RMDX::setCommunicationTimeout(unsigned char const& motorId, int32_t const& timeoutMS)
{
    CANMessage message = createMessage(motorId, MyActuator::commands::COMM_INTERRUPT_TIMEOUT, timeoutMS);
    return canReadWrite(message, false) == 0;
}


int RMDX::canReadWrite(CANMessage& message, bool const& waitForReply)
{
    CANMessage dump;
    while (canDriver_->isDataAvailable())
        canDriver_->readAvailableMessage(dump);
    canDriver_->sendMessage(message);

    if (waitForReply)
    {
        struct timespec startTime, currentTime;
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        while (!canDriver_->isDataAvailable())
        {
            usleep(100);
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            double const elapsed = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1e9;
            if (elapsed > timeout_)
                break;
        }

        if (true)
        // if (canDriver_->isDataAvailable())
        {
            canDriver_->readAvailableMessage(message);
            canDriver_->readAvailableMessage(message);
            return 1;
        }
        else
            return -2;
    }
    return 0;
}
}
