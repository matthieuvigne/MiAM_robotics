/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <fcntl.h>

#include <miam_utils/miam_utils.h>
#include "Experiment.h"

// Include local headers and definition of str2ba to prevent having to link to bluez library.
#include "bluetooth/bluetooth.h"
#include "bluetooth/rfcomm.h"

int bachk(const char *str)
{
    if (!str)
        return -1;

    if (strlen(str) != 17)
        return -1;

    while (*str) {
        if (!isxdigit(*str++))
            return -1;

        if (!isxdigit(*str++))
            return -1;

        if (*str == 0)
            break;

        if (*str++ != ':')
            return -1;
    }

    return 0;
}

int str2ba(const char *str, bdaddr_t *ba)
{
    int i;

    if (bachk(str) < 0) {
        memset(ba, 0, sizeof(*ba));
        return -1;
    }

    for (i = 5; i >= 0; i--, str += 3)
        ba->b[i] = strtol(str, NULL, 16);

    return 0;
}

Experiment::Experiment():
    port_(-1),
    isConnected_(false),
    isStarted_(false),
    shouldStart_(false)
{
    std::thread t(&Experiment::loop, this);
    t.detach();
}


bool Experiment::isConnected()
{
    mutex_.lock();
    bool connected = isConnected_;
    mutex_.unlock();
    return connected;
}


bool Experiment::hasStarted()
{
    bool result = false;
    mutex_.lock();
    result = isStarted_;
    mutex_.unlock();
    return result;
}


void Experiment::start()
{
    mutex_.lock();
    shouldStart_ = true;
    mutex_.unlock();
}


bool Experiment::checkConnection()
{
    if(port_ < 0)
        return false;
    // Send 'a', look if 'M' is the reply.
    if(write(port_, "a", 1) < 0)
        return false;

    // Check that return value is 'M', with 10ms timeout.
    // Read up to 10 bytes to flush buffer if needed.
    unsigned char reply[10];
    if(read_timeout(port_, reply, 10, 500) < 1)
    {
        // If errno is EINPROGRESS, just ignore error.
        if (errno != EINPROGRESS)
            return false;
    }
    return reply[0] == 'M';
}

bool Experiment::sendStart()
{
    if(port_ < 0)
        return false;
    // Send 's', look if 'S' is the reply.
    if(write(port_, "s", 1) < 0)
        return false;

    // Check that return value is 'S', with 10ms timeout.
    // Read up to 10 bytes to flush buffer if needed.
    unsigned char reply[10];
    if(read_timeout(port_, reply, 10, 500) < 1)
    {
        // If errno is EINPROGRESS, just ignore error.
        if (errno != EINPROGRESS)
            return false;
    }
    return reply[0] == 'S';
}


void Experiment::loop()
{
    // Main loop, running in a background thread.
    #ifdef DEBUG
        std::cout << "Running background thread" << std::endl;
    #endif
    // Start the connection.

     // Hard-coded destination.
    char dest[18] = "98:D3:71:F9:87:3E";

    // Loop while connection is not ok.
    while(port_ == -1)
    {
        std::cout << "trying to connect" << std::endl;

        // Create socket struct.
        struct sockaddr_rc addr = { 0 };
        addr.rc_family = AF_BLUETOOTH;
        addr.rc_channel = (uint8_t) 1;
        str2ba( dest, &addr.rc_bdaddr );

        // Create socket and try to connect with timeout.
        port_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

        int flags = fcntl(port_, F_GETFL, 0);
        fcntl(port_, F_SETFL, flags | O_NONBLOCK);

        // Try to connect with a 3s timeout
        int status = connect(port_, (struct sockaddr *)&addr, sizeof(addr));
        if (status < 0)
        {
            if (errno == EINPROGRESS || errno == EBUSY)
            {
                struct timeval timeout;
                timeout.tv_sec = 3;
                timeout.tv_usec = 0;
                fd_set set;
                FD_ZERO(&set);
                FD_SET(port_, &set);

                if(select(port_ + 1, NULL, &set, NULL, &timeout) > 0)
                {
                    // Set blocking mode again.
                    flags = fcntl(port_, F_GETFL, 0);
                    fcntl(port_, F_SETFL, flags & ~O_NONBLOCK);
                    // Check that there was no connection error.
                    socklen_t lon = sizeof(int);
                    int valopt = sizeof(int);
                    getsockopt(port_, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
                    if (valopt)
                    {
                        close(port_);
                        port_ = -1;
                    }
                }
                else
                {
                    close(port_);
                    port_ = -1;
                }
            }
            else
            {
                close(port_);
                port_ = -1;
            }
        }
        std::cout << "Bluetooth error:" << errno << std::endl;


        // Set blocking mode again.
        flags = fcntl(port_, F_GETFL, 0);
        fcntl(port_, F_SETFL, flags & ~O_NONBLOCK);
        if (port_ > -1)
        {
            flags = fcntl(port_, F_GETFL, 0);
            fcntl(port_, F_SETFL, flags & ~O_NONBLOCK);
        }
    }
    std::cout << "Experiment connected" << std::endl;

    // Main loop
    while(true)
    {
        // Check that connection is alive.
        bool connected = checkConnection();

        bool shouldStart;
        mutex_.lock();
        isConnected_ = connected;
        shouldStart = shouldStart_;
        mutex_.unlock();

        if (shouldStart)
        {
            bool started = sendStart();
            mutex_.lock();
            isStarted_ = started;
            mutex_.unlock();
            if (started)
                break;
        }
    }

    // Close connection.
    close(port_);
    #ifdef DEBUG
        std::cout << "Terminating bluetooth background thread" << std::endl;
    #endif
}
