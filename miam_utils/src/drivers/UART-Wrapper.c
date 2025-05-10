/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/UART-Wrapper.h"

#include <fcntl.h>
#include <unistd.h>
// Open a uart port.
int uart_open(std::string const& portName, int speed)
{
    // Open in read-write mode.
    int port = open(portName.c_str(), O_RDWR);
    if(port == -1)
        return port;

    // Setup UART port.
    struct termios options;
    tcgetattr(port, &options);

    //Set communication speed and options
    //8 bit, 1 stop, no parity
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~ECHO;

    // Set input and output to raw mode.
    cfmakeraw(&options);

    tcsetattr(port, TCSAFLUSH, &options);
    usleep(100000);
    tcflush(port, TCIOFLUSH);

    return port;
}

int read_timeout(int const& file, unsigned char *buffer, size_t const& size, double const& timeoutSec)
{

    struct timespec startTime, currentTime;
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    struct timeval timeout;
    timeout.tv_sec = static_cast<int>(timeoutSec);
    timeout.tv_usec = (timeoutSec - timeout.tv_sec) * 1e6;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(file, &set);

    int nRead = 0;
    while (true)
    {
        int nFiles = select(file + 1, &set, NULL, NULL, &timeout);

        // If there is something to read, read and return the number of bytes read.
        if (nFiles > 0)
        {
            int n =  read(file, buffer + nRead, size - nRead);
            nRead += n;
            if (nRead == size)
                return nRead;
        }
        else
        {
            // Nothing to read: return the return value of select: 0 if timeout, -1 on error.
            return nFiles;
        }

        // If we still have time, try again.
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        double elapsed = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1.0e9;
        if (elapsed > timeoutSec)
            return -1;

        timeout.tv_sec = static_cast<int>(timeoutSec - elapsed);
        timeout.tv_usec = (timeoutSec - elapsed - timeout.tv_sec) * 1e6;
    }
}
