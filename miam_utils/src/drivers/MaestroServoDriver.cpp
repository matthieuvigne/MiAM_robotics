/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/MaestroServoDriver.h"
#include "miam_utils/drivers/UART-Wrapper.h"

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>



MaestroDriver::MaestroDriver():
    port_(-1),
    deviceID_(0)
{
}


bool MaestroDriver::init(std::string const& portName, int const& deviceID)
{
    // Open port
    port_ = uart_open(portName, B115200);
    deviceID_ = deviceID;

    if(port_ == -1)
        return false;

    // Check that a Maestro servo driver is indeed present.
    // This is done by sending a GetMovingState command and checking the reply.
    tcflush(port_, TCIOFLUSH);

    unsigned char returnData[1];
    sendCommand(0x13, returnData, 1);
    int returnValue = read_timeout(port_, returnData, 1, 100);
    std::cout << returnValue << std::endl;
    if(returnValue < 1 || returnData[0] > 1)
        return false;

    return true;
}




void MaestroDriver::setPosition(int const& servo, double const& position)
{
    clearError();
    unsigned char parameters[3];
    parameters[0] = servo;
    // Command unit: 0.25us.
    int servoCommand = (int) floor(position * 4);
    if(servoCommand < 0)
        servoCommand = 0;
    if(servoCommand > 2500 * 4)
        servoCommand = 2500 * 4;
    parameters[1] = servoCommand & 0x7F;
    parameters[2] = (servoCommand >> 7) & 0x7F;
    sendCommand(0x04, parameters, 3);
}


void MaestroDriver::setSpeed(int const& servo, int const& speed)
{
    clearError();
    unsigned char parameters[3];
    parameters[0] = servo;
    // Command unit: 25 us/s
    int servoCommand = speed / 25;
    parameters[1] = servoCommand & 0xFF;
    parameters[2] = (servoCommand >> 8) & 0xFF;
    sendCommand(0x07, parameters, 3);
}


int MaestroDriver::sendCommand(int const& commandID, unsigned char *parameters, int const& length)
{
    unsigned char message[3 + length];
    message[0] = 0xAA;
    message[1] = deviceID_;
    message[2] = commandID;
    for(int i = 0; i < length; i++)
        message[3+i] = parameters[i];

    return write(port_, message, 3 + length);
}

void MaestroDriver::clearError()
{
    unsigned char param = 0;
    sendCommand(0x21, &param, 0);
}
