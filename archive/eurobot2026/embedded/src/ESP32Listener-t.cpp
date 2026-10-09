#include "ESP32Listener.h"

#include <iostream>
#include <unistd.h>


int main (int argc, char *argv[])
{
    ESP32Listener listener;
    if (!listener.init("/dev/ttyACM0", 1000000))
    {
        std::cout << "Failed to init communication" << std::endl;
    }

    while (true)
    {
        if (listener.update())
        {
            std::cout << listener.getLastData() << std::endl;
        }
        usleep(100000);
    }
}