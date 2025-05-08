/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/UART-Wrapper.h"
#include "miam_utils/raspberry_pi/RPiGPIO.h"
#include "miam_utils/drivers/STSServoDriver.h"

#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>

namespace instruction
{
    unsigned char const PING      = 0x01;
    unsigned char const READ      = 0x02;
    unsigned char const WRITE     = 0x03;
    unsigned char const REGWRITE  = 0x04;
    unsigned char const ACTION    = 0x05;
    unsigned char const SYNCWRITE = 0x83;
    unsigned char const RESET     = 0x06;
};

int16_t STS::radToServoValue(double const& rad)
{
    return static_cast<int16_t>(2048 + 2048 * rad / M_PI);
}

double STS::servoToRadValue(int16_t const& ticks)
{
    return static_cast<double>(M_PI * (ticks - 2048) / 2048);
}


STSServoDriver::STSServoDriver(double const& readTimeout):
    port_(-1),
    dirPin_(0),
    readTimeout_(readTimeout)
{
    for (int i = 0; i < 256; i++)
    {
        lastCommands_[(unsigned char) i] = -1;
        servoType_[i] = STS::ServoType::UNKNOWN;
    }
}


bool STSServoDriver::init(std::string const& portName, int const& dirPin, int const& baudRate)
{
    // Open port
    port_ = uart_open(portName, baudRate);
    dirPin_ = dirPin;
    if (dirPin_ > 0)
        RPi_setupGPIO(dirPin_, PI_GPIO_OUTPUT);

    if(port_ == -1)
        return false;

    // Test that a servo is present.
    bool hasPing = false;
    for (unsigned char i = 0; i < 0xFE; i++)
        if (ping(i))
        {
            hasPing = true;
            break;
        }
    if (hasPing)
    {
        for (int i = 0; i < 256; i++)
        {
            servoType_[i] = STS::ServoType::UNKNOWN;
        }
        // Configure servos
        writeRegister(0xFE, STS::registers::WRITE_LOCK, 0);
        usleep(2000);
        // Set a return delay of 4us.
        writeRegister(0xFE, STS::registers::RESPONSE_DELAY, 2);
        usleep(2000);
        // Set a return status level of 0.
        writeRegister(0xFE, STS::registers::RESPONSE_STATUS_LEVEL, 0);
        usleep(2000);
        // Set voltage limit to  13V
        writeRegister(0xFE, STS::registers::MAXIMUM_VOLTAGE, 130);
        usleep(2000);
        // Set all protections on
        writeRegister(0xFE, STS::registers::UNLOADING_CONDITION, 44);
        usleep(2000);
        // Lock EEPROM
        writeRegister(0xFE, STS::registers::WRITE_LOCK, 1);
        usleep(2000);
        writeRegister(0xFE, STS::registers::MINIMUM_ANGLE, 0);
        usleep(2000);
        writeRegister(0xFE, STS::registers::MINIMUM_ANGLE + 1, 0);
        usleep(2000);
        writeRegister(0xFE, STS::registers::MINIMUM_ANGLE + 2, 0);
        usleep(2000);
        writeRegister(0xFE, STS::registers::MINIMUM_ANGLE + 3, 0);
        usleep(2000);
        // Disable all servos
        disable(0xFE);

        // Give some time for the servos to configure themselves.
        usleep(20000);
        return true;
    }
    return false;
}


bool STSServoDriver::ping(unsigned char const& servoId)
{
    tcflush(port_, TCIOFLUSH);
    usleep(100);
    tcflush(port_, TCIOFLUSH);

    unsigned char response[1] = {0xFF};
    int send = sendMessage(servoId,
                           instruction::PING,
                           0,
                           response,
                           true);
    // Failed to send
    if (send != 6)
        return false;
    // Read response
    int rd = recieveMessage(servoId, 1, response);
    if (rd < 0)
        return false;
    return response[0] == 0x00;
}


std::vector<unsigned char> STSServoDriver::detectServos()
{
    std::vector<unsigned char> ids;
    for (unsigned char i = 0; i < 0xFE; i++)
    {
        // Reset detected types, servo might have changed.
        servoType_[i] = STS::ServoType::UNKNOWN;
        if (ping(i))
            ids.push_back(i);
    }
    return ids;
}

bool STSServoDriver::setMode(unsigned char const& servoId, STS::Mode const& mode)
{
    bool rc = writeRegister(servoId, STS::registers::OPERATION_MODE, static_cast<unsigned char>(mode));
    if (rc && mode == STS::Mode::STEP)
        rc = writeTwoBytesRegister(servoId, STS::registers::MAXIMUM_ANGLE, 0);
    else if (rc && mode == STS::Mode::POSITION)
        rc = writeTwoBytesRegister(servoId, STS::registers::MAXIMUM_ANGLE, 4095);

    return rc;
}


bool STSServoDriver::disable(unsigned char const& servoId, bool const& disable)
{
    return writeRegister(servoId, STS::registers::TORQUE_SWITCH, disable ? 0 : 1);
}


bool STSServoDriver::resetPositionAsCenter(unsigned char const& servoId)
{
    return writeRegister(servoId, STS::registers::TORQUE_SWITCH, 128);
}


bool STSServoDriver::setId(unsigned char const& oldServoId, unsigned char const& newServoId)
{
    if (servoType_[oldServoId] == STS::ServoType::UNKNOWN)
    {
        determineServoType(oldServoId);
    }

    if (oldServoId >= 0xFE || newServoId >= 0xFE)
        return false;
    if (ping(newServoId))
        return false; // address taken
    unsigned char lockRegister = STS::registers::WRITE_LOCK;
    if (servoType_[oldServoId] == STS::ServoType::SCS)
    {
        lockRegister = STS::registers::TORQUE_LIMIT; // On SCS, this has been remapped.
    }

    if (!writeRegister(oldServoId, lockRegister, 0))
        return false;
    usleep(5000);
    // Write new ID
    if (!writeRegister(oldServoId, STS::registers::ID, newServoId))
        return false;
    usleep(50000);
    // Lock EEPROM
    if (!writeRegister(newServoId, lockRegister, 1))
      return false;
    // Give it some time to change id.
    usleep(500000);
    // Update servo type cache.
    servoType_[newServoId] = servoType_[oldServoId];
    servoType_[oldServoId] = STS::ServoType::UNKNOWN;
    return ping(newServoId);
}


int16_t STSServoDriver::getCurrentPosition(unsigned char const& servoId)
{
    return readTwoBytesRegister(servoId, STS::registers::CURRENT_POSITION);
}

int16_t STSServoDriver::getLastCommand(unsigned char const& servoId)
{
    return lastCommands_[servoId];
}


int16_t STSServoDriver::getCurrentSpeed(unsigned char const& servoId)
{
    return readTwoBytesRegister(servoId, STS::registers::CURRENT_SPEED);
}


double STSServoDriver::getCurrentTemperature(unsigned char const& servoId)
{
    return readTwoBytesRegister(servoId, STS::registers::CURRENT_TEMPERATURE);
}


double STSServoDriver::getCurrentCurrent(unsigned char const& servoId)
{
    int16_t current = readTwoBytesRegister(servoId, STS::registers::CURRENT_CURRENT);
    return current * 0.0065;
}

bool STSServoDriver::isMoving(unsigned char const& servoId)
{
    unsigned char result = readRegister(servoId, STS::registers::MOVING_STATUS);
    // Try a second time on failure
    if (returnCode_ < 0)
        result = readRegister(servoId, STS::registers::MOVING_STATUS);
    return result > 0;
}


void STSServoDriver::setPIDGains(unsigned char const& servoId, unsigned char const& Kp, unsigned char const& Kd, unsigned char const& Ki)
{
    writeRegister(servoId, STS::registers::WRITE_LOCK, 0);
    writeRegister(servoId, STS::registers::POS_PROPORTIONAL_GAIN, Kp);
    writeRegister(servoId, STS::registers::POS_DERIVATIVE_GAIN, Kd);
    writeRegister(servoId, STS::registers::POS_INTEGRAL_GAIN, Ki);
    writeRegister(servoId, STS::registers::WRITE_LOCK, 1);
}


void STSServoDriver::setTorqueLimit(unsigned char const& servoId, double const& torqueLimit)
{
    writeRegister(servoId, STS::registers::WRITE_LOCK, 0);
    writeRegister(servoId, STS::registers::TORQUE_LIMIT, static_cast<int>(1000 * std::clamp(torqueLimit, 0.0, 1.0)));
    writeRegister(servoId, STS::registers::MAXIMUM_TORQUE, static_cast<int>(1000 * std::clamp(torqueLimit, 0.0, 1.0)));
    writeRegister(servoId, STS::registers::WRITE_LOCK, 1);
}


bool STSServoDriver::setTargetPosition(unsigned char const& servoId, int16_t const& position, bool const& asynchronous)
{
    lastCommands_[servoId] = position;
    return writeTwoBytesRegister(servoId, STS::registers::TARGET_POSITION, position, asynchronous);
}


bool STSServoDriver::setTargetVelocity(unsigned char const& servoId, int16_t const& velocity, bool const& asynchronous)
{
    if (servoType_[servoId] == STS::ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }
    switch(servoType_[servoId])
    {
        case STS::ServoType::STS:
            return writeTwoBytesRegister(servoId, STS::registers::RUNNING_SPEED, velocity, asynchronous);
        case STS::ServoType::SCS:
            // Not implemented, this messes us the servo configuration and is not used for now anyway.
            return false;
            // writeRegister(servoId, STS::registers::MINIMUM_ANGLE, 0);
            // usleep(2000);
            // writeRegister(servoId, STS::registers::MINIMUM_ANGLE + 1, 0);
            // usleep(2000);
            // writeRegister(servoId, STS::registers::MINIMUM_ANGLE + 2, 0);
            // usleep(2000);
            // writeRegister(servoId, STS::registers::MINIMUM_ANGLE + 3, 0);
            // usleep(2000);
            // return writeTwoBytesRegister(servoId, STS::registers::RUNNING_TIME, velocity, asynchronous);
        default:
            return false;
    }
}


bool STSServoDriver::trigerAction()
{
    unsigned char noParam = 0;
    int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
    return send == 6;
}


int STSServoDriver::sendMessage(unsigned char const& servoId,
                                unsigned char const& commandID,
                                unsigned char const& paramLength,
                                unsigned char *parameters,
                                bool const& willRead)
{
    if (port_ < 0)
        return 0;
    if (!willRead)
        mutex_.lock();
    unsigned char message[6 + paramLength];
    unsigned char checksum = servoId + paramLength + 2 + commandID;
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = servoId;
    message[3] = paramLength + 2;
    message[4] = commandID;
    for (int i = 0; i < paramLength; i++)
    {
        message[5 + i] = parameters[i];
        checksum += parameters[i];
    }
    message[5 + paramLength] = ~checksum;

    if (dirPin_ > 0)
    {
        RPi_writeGPIO(dirPin_, false);
        usleep(10);
    }

    int ret = write(port_, message, 6 + paramLength);
    tcflush(port_, TCOFLUSH);
    if (dirPin_ > 0 && !willRead)
    {
        usleep(10);
        RPi_writeGPIO(dirPin_, true);
    }
    if (!willRead)
    {
        usleep(50);
        mutex_.unlock();
    }
    return ret;
}


bool STSServoDriver::writeRegisters(unsigned char const& servoId,
                                    unsigned char const& startRegister,
                                    unsigned char const& writeLength,
                                    unsigned char const *parameters,
                                    bool const& asynchronous)
{
    unsigned char param[writeLength + 1];
    param[0] = startRegister;
    for (int i = 0; i < writeLength; i++)
        param[i + 1] = parameters[i];
    int rc =  sendMessage(servoId,
                          asynchronous ? instruction::REGWRITE : instruction::WRITE,
                          writeLength + 1,
                          param);
    return rc == writeLength + 7;
}


bool STSServoDriver::writeRegister(unsigned char const& servoId,
                                   unsigned char const& registerId,
                                   unsigned char const& value,
                                   bool const& asynchronous)
{
    return writeRegisters(servoId, registerId, 1, &value, asynchronous);
}


bool STSServoDriver::writeTwoBytesRegister(unsigned char const& servoId,
                                           unsigned char const& registerId,
                                           int16_t const& value,
                                           bool const& asynchronous)
{
    uint16_t servoValue = 0;

    if (servoType_[servoId] == STS::ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }
    switch(servoType_[servoId])
    {
        case STS::ServoType::SCS:
            // Little endian ; byte 10 is sign.
            servoValue = std::abs(value);
            if (value < 0)
                servoValue = 0x0400  | servoValue;
            // Invert endianness
            servoValue = (servoValue >> 8) + ((servoValue & 0xFF) << 8);
            break;
        case STS::ServoType::STS:
        default:
            servoValue = std::abs(value);
            if (value < 0)
                servoValue = 0x8000  | servoValue;
            break;
    }
    unsigned char params[2] = {static_cast<unsigned char>(servoValue & 0xFF),
                               static_cast<unsigned char>((servoValue >> 8) & 0xFF)};
    return writeRegisters(servoId, registerId, 2, params, asynchronous);
}


unsigned char STSServoDriver::readRegister(unsigned char const& servoId, unsigned char const& registerId)
{
    unsigned char result = 0;
    returnCode_= readRegisters(servoId, registerId, 1, &result);
    if (returnCode_ < 0)
        return 0;
    return result;
}


int16_t STSServoDriver::readTwoBytesRegister(unsigned char const& servoId, unsigned char const& registerId)
{

    if (servoType_[servoId] == STS::ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    unsigned char result[2] = {0, 0};
    int16_t value = 0;
    int16_t signedValue = 0;
    returnCode_ = readRegisters(servoId, registerId, 2, result);
    if (returnCode_ < 0)
        return 0;
    switch(servoType_[servoId])
    {
        case STS::ServoType::SCS:
            value = static_cast<int16_t>(result[1] +  (result[0] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        case STS::ServoType::STS:
            value = static_cast<int16_t>(result[0] +  (result[1] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        default:
            return 0;
    }
}


int STSServoDriver::readRegisters(unsigned char const& servoId,
                                  unsigned char const& startRegister,
                                  unsigned char const& readLength,
                                  unsigned char *outputBuffer)
{
    #define N_RETRIES 5

    static int success = 0;
    static int fail = 0;
    mutex_.lock();
    for (int i = 0; i < N_RETRIES; i++)
    {
        tcflush(port_, TCIFLUSH);
        unsigned char readParam[2] = {startRegister, readLength};
        int send = sendMessage(servoId, instruction::READ, 2, readParam, true);
        // Failed to send
        if (send != 8)
            continue;
        // Read
        unsigned char result[readLength + 1];
        int rd = recieveMessage(servoId, readLength + 1, result);
        if (rd < 0)
        {
            usleep(200);
            continue;
        }

        for (int i = 0; i < readLength; i++)
            outputBuffer[i] = result[i + 1];
        mutex_.unlock();
        success++;
        return 0;
    }
    fail ++;
#ifdef DEBUG
    std::cout << "[STS servo] Failed to read register " << success << " " << fail << std::endl;
#endif
    mutex_.unlock();
    return -1;
}

int STSServoDriver::recieveMessage(unsigned char const& servoId,
                                   unsigned char const& readLength,
                                   unsigned char *outputBuffer)
{
    if (dirPin_ > 0)
        RPi_writeGPIO(dirPin_, true);
    unsigned char result[readLength + 5];
    int rd = read_timeout(port_, result, readLength + 5, readTimeout_);
    if (rd != readLength + 5)
        return -1;
    // Check message integrity
    if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId || result[3] != readLength + 1)
        return -2;
    unsigned char checksum = 0;
    for (int i = 2; i < readLength + 4; i++)
        checksum += result[i];
    checksum = ~checksum;
    if (result[readLength + 4] != checksum)
        return -3;

    // Copy result to output buffer
    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 4];
    return 0;
}

void STSServoDriver::determineServoType(unsigned char const& servoId)
{
    switch(readRegister(servoId, STS::registers::SERVO_MAJOR))
    {
        case 9: servoType_[servoId] = STS::ServoType::STS; break;
        case 5: servoType_[servoId] = STS::ServoType::SCS; break;
        default: servoType_[servoId] = STS::ServoType::STS; break;
    }
}
