// Example of reading vision shared memory

#include "VisionSharedMem.h"

#include <iostream>

int main(int argc, char *argv[])
{
    SHMReader reader;
    if (!reader.init())
    {
        std::cout << "Failed read!" << std::endl;
        return -1;
    }

    VisionBuffer buffer;

    while (true)
    {
        usleep(100000);

        reader.update(buffer);
        std::cout << "Read shared memory: found" << buffer.nMarkers << " markers." << std::endl;
        for (int i = 0; i < buffer.nMarkers; i++)
            std::cout << " - " << buffer.markers[i].height << " " << buffer.markers[i].radius << " " << buffer.markers[i].theta_rad << std::endl;
    }
}
