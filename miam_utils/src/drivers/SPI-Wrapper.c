/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
 #include "MiAMEurobot/drivers/SPI-Wrapper.h"
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

int spi_open(std::string const& portName, int const& frequency)
{

    int port = open(portName.c_str(), O_RDWR) ;
    if(port < 0)
    {
        #ifdef DEBUG
            printf("Error opening SPI bus %s %d\n", portName, errno);
        #endif
        return -1 ;
    }

    // Setup the SPI bus, mode 3 (SPI_CPOL | SPI_CPHA).
    // See documentation of spi/spidev for details.
    int mode = SPI_MODE_3;
    if(ioctl(port, SPI_IOC_WR_MODE, &mode) < 0)
    {
        #ifdef DEBUG
            printf("Error configuring port %s %d\n", portName, errno);
        #endif
        return -1 ;
    }

    int nBits = 8;
    if(ioctl(port, SPI_IOC_WR_BITS_PER_WORD, &nBits) < 0)
    {
        #ifdef DEBUG
            printf("Error configuring port %s %d\n", portName, errno);
        #endif
        return -1 ;
    }

    if(ioctl(port, SPI_IOC_WR_MAX_SPEED_HZ, &frequency)   < 0)
    {
        #ifdef DEBUG
            printf("Error configuring port %s %d\n", portName, errno);
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
                printf("Error closing SPI port: %d\n", errno);
            #endif
        }
        close(port);
    }
}
