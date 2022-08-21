/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/SPI-Wrapper.h"
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <cstring>

SPIWrapper::SPIWrapper(std::string const& portName, int const& frequency):
    portName_(portName),
    fd_(-1),
    frequency_(frequency),
    mutex_()
{

}


int SPIWrapper::spiReadWriteSingle(uint8_t* data, uint8_t const& len)
{
    static struct spi_ioc_transfer spiCtrl;

    // First element: send address and wait.
    spiCtrl.tx_buf = (unsigned long)&data[0];
    spiCtrl.rx_buf = (unsigned long)&data[0];
    spiCtrl.len = len;
    spiCtrl.speed_hz = frequency_;
    spiCtrl.bits_per_word = 8;
    spiCtrl.delay_usecs = 0;
    spiCtrl.cs_change = true;

    return spiReadWrite(1, &spiCtrl);
}

int SPIWrapper::spiReadWrite(uint8_t const& numberOfPackets, struct spi_ioc_transfer* spiCtrl)
{
    for (int i = 0; i < numberOfPackets; i++)
    {
        spiCtrl[i].speed_hz = frequency_;
        spiCtrl[i].delay_usecs = static_cast<uint16_t>(2e6 / frequency_) + 1;
    }
    mutex_.lock();
    spi_open();
    int res = ioctl(fd_, SPI_IOC_MESSAGE(numberOfPackets), spiCtrl);
    spi_close();
    mutex_.unlock();
    return res;
}


void SPIWrapper::spi_open()
{
    fd_ = open(portName_.c_str(), O_RDWR) ;
    if(fd_ < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error opening bus " << fd_ << ": " << std::strerror(errno) << std::endl;
        #endif
        return;
    }

    // Setup the SPI bus, mode 3 (SPI_CPOL | SPI_CPHA).
    // See documentation of spi/spidev for details.
    int mode = SPI_MODE_3;
    if(ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd_ " << fd_ << ": " << std::strerror(errno) << std::endl;
        #endif
        return;
    }

    int nBits = 8;
    if(ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &nBits) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd_ " << fd_ << ": " << std::strerror(errno) << std::endl;
        #endif
        return;
    }

    if(ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &frequency_)   < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd_ " << fd_ << ": " << std::strerror(errno) << std::endl;
        #endif
        return;
    }
    return;
}


void SPIWrapper::spi_close()
{
    if(fd_ > 0)
    {
        // Release CS - for some reason this is needed for the raspberry pi, as CS is not automatically released.
        int mode = SPI_MODE_0;
        if(ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0)
        {
            #ifdef DEBUG
            std::cout << "SPI error closing fd_ " << fd_ << ": " << std::strerror(errno) << std::endl;
            #endif
        }
        close(fd_);
    }
}
