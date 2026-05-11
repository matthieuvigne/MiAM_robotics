#include <miam_utils/COBSSerialListener.h>
#include <miam_utils/COBS.h>
#include <miam_utils/drivers/UART-Wrapper.h>

#include <iostream>
#include <unistd.h>

#define TERMINATION_CHAR 0x00

int constexpr MAX_SIZE = 256;

CobsSerialListener::CobsSerialListener()
{
    inputBuffer_.reserve(MAX_SIZE);
    outputBuffer_.reserve(MAX_SIZE);
}

bool CobsSerialListener::init(std::string const& portName, int const& baudRate)
{
    port_ = uart_open(portName, baudRate, true);
    if (port_ >= 0)
    {

        tcflush(port_, TCIOFLUSH);
    }
    return port_ >= 0;
}

bool CobsSerialListener::update()
{
    if (port_ < 0)
        return false;
    bool result = false;
    uint8_t buffer[MAX_SIZE];
    int nRead = read(port_, buffer, MAX_SIZE);

    for (int i = 0; i < nRead; i++)
    {
        uint8_t incomingByte = buffer[i];
        if (incomingByte == TERMINATION_CHAR)
        {
            uint8_t decoded[MAX_SIZE];
            int nOut = COBS::decode(inputBuffer_.data(), inputBuffer_.size(), decoded);
            outputBuffer_.clear();
            for (int i = 0; i < nOut; i++)
                outputBuffer_.push_back(decoded[i]);
            inputBuffer_.clear();
            result = true;
        }
        else
        {
            if (inputBuffer_.size() < MAX_SIZE)
                inputBuffer_.push_back(incomingByte);
        }
    }
    return result;
}

std::vector<uint8_t> CobsSerialListener::getLastFrame()
{
    return outputBuffer_;
}
