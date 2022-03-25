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

int spi_open(std::string const& portName, int const& frequency)
{

    int port = open(portName.c_str(), O_RDWR) ;
    if(port < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error opening bus " << portName << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1 ;
    }

    // Setup the SPI bus, mode 3 (SPI_CPOL | SPI_CPHA).
    // See documentation of spi/spidev for details.
    int mode = SPI_MODE_3;
    if(ioctl(port, SPI_IOC_WR_MODE, &mode) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring port " << portName << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1 ;
    }

    int nBits = 8;
    if(ioctl(port, SPI_IOC_WR_BITS_PER_WORD, &nBits) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring port " << portName << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1 ;
    }

    if(ioctl(port, SPI_IOC_WR_MAX_SPEED_HZ, &frequency)   < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring port " << portName << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1 ;
    }
    return port;
}


void spi_close(int const& port)
{
    if(port > 0)
    {
        // Release CS - for some reason this is needed for the raspberry pi, as CS is not automatically released.
        int mode = SPI_MODE_0;
        if(ioctl(port, SPI_IOC_WR_MODE, &mode) < 0)
        {
            #ifdef DEBUG
            std::cout << "SPI error closing port " << port << ": " << std::strerror(errno) << std::endl;
            #endif
        }
        close(port);
    }
}
