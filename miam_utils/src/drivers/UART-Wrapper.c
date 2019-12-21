/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/drivers/UART-Wrapper.h"

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

int read_timeout(int const& file, unsigned char *buffer, size_t const& size, uint const& timeoutMs)
{
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000 * timeoutMs;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(file, &set);

    if(select(file + 1, &set, NULL, NULL, &timeout) > 0)
        return read(file, buffer, size);
    return -1;
}
