/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/UART-Wrapper.h"
#include "miam_utils/raspberry_pi/RPiGPIO.h"
#include "miam_utils/drivers/STSServoDriver.h"

#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

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


STSServoDriver::STSServoDriver(double const& readTimeout):
    port_(-1),
    dirPin_(0),
    readTimeout_(readTimeout)
{
}


bool STSServoDriver::init(std::string const& portName, int const& dirPin, int const& baudRate)
{
    // Open port
    port_ = uart_open(portName, baudRate);
    dirPin_ = dirPin;
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
        // Configure servos
        writeRegister(0xFE, STS::registers::WRITE_LOCK, 0);
        // Set a return delay of 20ms.
        writeRegister(0xFE, STS::registers::RESPONSE_DELAY, 10);
        // Set a return status level of 0.
        writeRegister(0xFE, STS::registers::RESPONSE_STATUS_LEVEL, 0);
        // Set voltage limit to  8.5V
        writeRegister(0xFE, STS::registers::MAXIMUM_VOLTAGE, 85);
        // Set all protections on
        writeRegister(0xFE, STS::registers::UNLOADING_CONDITION, 44);
        // Lock EEPROM
        writeRegister(0xFE, STS::registers::WRITE_LOCK, 1);
        // Disable all servos
        disable(0xFE);

        // Give some time for the servos to configure themselves.
        usleep(10000);
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
                           response);
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
        if (ping(i))
            ids.push_back(i);
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
    if (oldServoId >= 0xFE || newServoId >= 0xFE)
        return false;
    if (ping(newServoId))
        return false; // address taken
    if (!writeRegister(oldServoId, STS::registers::WRITE_LOCK, 0))
        return false;
    // Write new ID
    if (!writeRegister(oldServoId, STS::registers::ID, newServoId))
        return false;
    // Lock EEPROM
    if (!writeRegister(newServoId, STS::registers::WRITE_LOCK, 1))
      return false;
    // Give it some time to change id.
    usleep(50000);
    return ping(newServoId);
}


int16_t STSServoDriver::getCurrentPosition(unsigned char const& servoId)
{
    return readTwoBytesRegister(servoId, STS::registers::CURRENT_POSITION);
}


int16_t STSServoDriver::getCurrentSpeed(unsigned char const& servoId)
{
    int16_t value = readTwoBytesRegister(servoId, STS::registers::CURRENT_SPEED);
    // Bit15 is sign bit
    int16_t signedValue = value & ~0x8000;
    if (value & 0x8000)
        signedValue = -signedValue;
    return signedValue;
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
    unsigned char const result = readRegister(servoId, STS::registers::MOVING_STATUS);
    return result > 0;
}


bool STSServoDriver::setTargetPosition(unsigned char const& servoId, int16_t const& position, bool const& asynchronous)
{
    // Bit 15 is the sign bit: change convention accordingly.
    uint16_t pos = std::abs(position);
    if (position < 0)
        pos = 0x8000  | pos;
    return writeTwoBytesRegister(servoId, STS::registers::TARGET_POSITION, pos, asynchronous);
}


bool STSServoDriver::setTargetVelocity(unsigned char const& servoId, int16_t const& velocity, bool const& asynchronous)
{
    // Bit 15 is the sign bit: change convention accordingly.
    uint16_t vel = std::abs(velocity);
    if (velocity < 0)
        vel = 0x8000  | vel;
    return writeTwoBytesRegister(servoId, STS::registers::RUNNING_SPEED, vel, asynchronous);
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
                                unsigned char *parameters)
{
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

    RPi_writeGPIO(dirPin_, false);
    // std::cout << "Sending" << std::endl;
    // for (int i = 0; i < 6 + paramLength; i++)
    //     std::cout <<  std::hex << int(message[i]) << " ";
    // std::cout << std::endl;
    int ret = write(port_, message, 6 + paramLength);
    RPi_writeGPIO(dirPin_, false);
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
                                           uint16_t const& value,
                                           bool const& asynchronous)
{
    unsigned char params[2] = {static_cast<unsigned char>(value & 0xFF),
                               static_cast<unsigned char>((value >> 8) & 0xFF)};
    return writeRegisters(servoId, registerId, 2, params, asynchronous);
}


unsigned char STSServoDriver::readRegister(unsigned char const& servoId, unsigned char const& registerId)
{
    unsigned char result = 0;
    int rc = readRegisters(servoId, registerId, 1, &result);
    if (rc < 0)
        return 0;
    return result;
}


int16_t STSServoDriver::readTwoBytesRegister(unsigned char const& servoId, unsigned char const& registerId)
{
    unsigned char result[2] = {0, 0};
    int rc = readRegisters(servoId, registerId, 2, result);
    if (rc < 0)
    {
        std::cout << "Failed to read" << std::endl;
        return 0;
    }
    return static_cast<int16_t>(result[0] +  (result[1] << 8));
}


int STSServoDriver::readRegisters(unsigned char const& servoId,
                                  unsigned char const& startRegister,
                                  unsigned char const& readLength,
                                  unsigned char *outputBuffer)
{
    tcflush(port_, TCIOFLUSH);
    unsigned char readParam[2] = {startRegister, readLength};
    int send = sendMessage(servoId, instruction::READ, 2, readParam);
    // Failed to send
    if (send != 8)
        return -1;
    // Read
    unsigned char result[readLength + 1];
    int rd = recieveMessage(servoId, readLength + 1, result);
    if (rd < 0)
        return rd;

    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 1];
    return 0;
}

int STSServoDriver::recieveMessage(unsigned char const& servoId,
                                   unsigned char const& readLength,
                                   unsigned char *outputBuffer)
{
    // std::cout << "trying to read a message:" << int(readLength) << std::endl;
    RPi_writeGPIO(dirPin_, false);
    usleep(300);
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