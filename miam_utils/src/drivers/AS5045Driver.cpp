#include "miam_utils/drivers/AS5045Driver.h"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cmath>
#include <iostream>
#include <errno.h>
#include <cstring>

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
    currentPosition_(0),
    previousSingleTurnPos_(0)
{

}

AS5045::AS5045(SPIWrapper *spiDriver):
    spiDriver_(spiDriver),
    isInit_(false),
    currentPosition_(0),
    previousSingleTurnPos_(0)
{

}

bool AS5045::init()
{
    currentPosition_ = 0.0;
    uint16_t pos = 0;
    isInit_ = readSPI(previousSingleTurnPos_);
    return isInit_;
}

double AS5045::updatePosition()
{
    if (isInit_)
    {
        double newPos = 0;
        if (readSPI(newPos))
        {
            // Continuity at position jump.
            double continuity = 0;
            if (newPos < M_PI_2 && previousSingleTurnPos_ > 3 * M_PI_2)
                continuity = 2 * M_PI;
            else if (newPos > 3 * M_PI_2 && previousSingleTurnPos_ < M_PI_2)
                continuity = -2 * M_PI;
            currentPosition_ += (newPos - previousSingleTurnPos_) + continuity;
            previousSingleTurnPos_ = newPos;
        }
    }
    return currentPosition_;
}


double AS5045::getPosition() const
{
    return currentPosition_;
}


bool AS5045::readSPI(double & position) const
{
    uint32_t data = 0;
    static struct spi_ioc_transfer spiCtrl;
    spiCtrl.tx_buf = (unsigned long)&data;
    spiCtrl.rx_buf = (unsigned long)&data;
    spiCtrl.len = 3;
    spiCtrl.delay_usecs = 1;
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
    // First bit is invalid (probably due to SPI timing) so everything is shift by one.
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
        position = static_cast<double>(rawPos) / 2048.0 * M_PI;
    else
        position = previousSingleTurnPos_;

    return result == 3 && isValid;
}