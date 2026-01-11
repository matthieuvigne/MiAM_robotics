

#include "uCListener.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    uCListener_start("/dev/ttyACM0");

    while(true)
    {
        std::cout << uCListener_getData() << std::endl;
        usleep(5000);
    }

    return 0;
}

