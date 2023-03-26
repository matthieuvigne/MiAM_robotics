#include "miam_utils/drivers/AS5045Driver.h"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cmath>
#include <iostream>
#include <errno.h>
#include <cstring>

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

// Utility function: check parity
bool check_parity(uint32_t n)
{
    uint32_t b;
    //Left Shifting by double of previous operation shift
    //And take xor with the previous result.
    b = n ^ (n >> 1);
    b = b ^ (b >> 2);
    b = b ^ (b >> 4);
    b = b ^ (b >> 8);
    b = b ^ (b >> 16);
    //Now b contain the parity value
    //If b is odd mean odd parity else even parity
    if (b & 1) //This is used for checking odd
        return 1;
    else
        return 0;
}

AS5045::AS5045():
    spiDriver_(nullptr),
    isInit_(false),
    currentPosition_(),
    previousSingleTurnPos_()
{

}

AS5045::AS5045(SPIWrapper *spiDriver, uint32_t const& nEncoders):
    spiDriver_(spiDriver),
    isInit_(false)
{
    for (uint32_t i = 0; i < nEncoders; i++)
    {
        currentPosition_.push_back(0);
        previousSingleTurnPos_.push_back(0);
    }
}

bool AS5045::init()
{
    std::fill(currentPosition_.begin(), currentPosition_.end(), 0);
    isInit_ = readSPI(previousSingleTurnPos_);
    isInit_ &= wasLastFrameValid_;
    return isInit_;
}

std::vector<double> AS5045::updatePosition()
{
    if (isInit_)
    {
        std::vector<double> newPos;
        for (uint32_t i = 0; i < currentPosition_.size(); i++)
            newPos.push_back(0);

        if (readSPI(newPos))
        {
            for (uint32_t i = 0; i < currentPosition_.size(); i++)
            {
                // Continuity at position jump.
                double continuity = 0;
                if (newPos.at(i) < M_PI_2 && previousSingleTurnPos_.at(i) > 3 * M_PI_2)
                    continuity = 2 * M_PI;
                else if (newPos.at(i) > 3 * M_PI_2 && previousSingleTurnPos_.at(i) < M_PI_2)
                    continuity = -2 * M_PI;
                currentPosition_.at(i) += (newPos.at(i) - previousSingleTurnPos_.at(i)) + continuity;
                previousSingleTurnPos_.at(i) = newPos.at(i);
            }
        }
    }
    return currentPosition_;
}


std::vector<double> AS5045::getPosition() const
{
    return currentPosition_;
}

// Decode encoder frame, return angle in rad, -1 on failure.
double decodeFrame(uint32_t const& data)
{
    uint16_t const rawPos= ((data & 0xFF) << 5) + (((data >> 8) & 0xF8) >> 3);

    bool const ofc = data & (1 << 10);
    bool const cof = data & (1 << 9);
    bool const lin = data & (1 << 8);
    bool const mag_inc = data & (1 << 23);
    bool const mag_dec = data & (1 << 22);
    bool const par = data & (1 << 21);
    bool const parity_check = !check_parity(data);

    bool const isValid = ofc && !cof && !lin && parity_check;

    #ifdef DEBUG
        if (!isValid)
        {
            std::cout << "AS5045 got invalid frame: OFC " << ofc << " cof" << cof << " lin" << lin;
            std::cout << " mag_inc" << mag_inc << " mag_dec" << " parity_check" << parity_check << std::endl;
        }
        if (mag_inc && mag_dec)
            std::cout << "AS5045 warning: magnetic increase and decrease detected" << std::endl;
        else
        {
            if (mag_inc)
                std::cout << "AS5045 warning: magnetic increase detected" << std::endl;
            if (mag_dec)
                std::cout << "AS5045 warning: magnetic decrease detected" << std::endl;
        }
    #endif

    if (isValid)
        return static_cast<double>(rawPos) / 2048.0 * M_PI;
    return -1;
}


bool AS5045::readSPI(std::vector<double> & position)
{
    uint32_t const nEncoders = currentPosition_.size();
    uint32_t const len = std::ceil((19.0 * nEncoders) / 8.0);

    uint8_t data[len];
    for (uint32_t i = 0; i < len; i++)
        data[i] = 0;

    static struct spi_ioc_transfer spiCtrl;
    spiCtrl.tx_buf = (unsigned long)&data;
    spiCtrl.rx_buf = (unsigned long)&data;
    spiCtrl.len = len;
    spiCtrl.delay_usecs = 0;
    spiCtrl.bits_per_word = 8;
    spiCtrl.cs_change = true;
    spiCtrl.pad = 0;
    spiCtrl.tx_nbits = 0;
    spiCtrl.rx_nbits = 0;

    int result = spiDriver_->spiReadWrite(1, &spiCtrl);

    if(result < 0)
    {
        #ifdef DEBUG
                std::cout << "AS5045 SPI error: " << errno << " " << std::strerror(errno) << std::endl;
        #endif
    }

    wasLastFrameValid_ = true;
    // Decompose message into streams of 19 bits and decode them
    for (uint32_t i = 0; i < nEncoders; i++)
    {
        // Find which bytes we need
        uint8_t const startByte = std::floor(19 * i / 8);
        uint32_t currentData = data[startByte] + (data[startByte + 1] << 8) + (data[startByte + 2] << 16);
        // Offset to get the correct LSB.
        uint8_t const lsbOffset = 19 * i - 8 * startByte;

        // Reverse the bits in each byte
        currentData = reverse(currentData & 0xFF) + (reverse((currentData >> 8) & 0xFF) << 8)+ (reverse((currentData >> 16) & 0xFF) << 16);

        // Shift, and mask everything except the bits [1 - 18]
        currentData = (currentData >> lsbOffset) & 0x0FFFFE;

        // Reverse again
        currentData = reverse(currentData & 0xFF) + (reverse((currentData >> 8) & 0xFF) << 8)+ (reverse((currentData >> 16) & 0xFF) << 16);

        // Decode single encoder frame.
        double pos = decodeFrame(currentData);
        if (pos >= 0)
            position.at(i) = pos;
        else
        {
            wasLastFrameValid_ = false;
            position.at(i) = previousSingleTurnPos_.at(i);
        }
    }

    return result == static_cast<int32_t>(len);
}