#include <miam_utils/COBSSerialListener.h>

#include <iostream>
#include <unistd.h>


int main (int argc, char *argv[])
{
    CobsSerialListener listener;
    if (!listener.init("/dev/ttyACM0", 1000000))
    {
        std::cout << "Failed to init communication" << std::endl;
    }

    while (true)
    {
        if (listener.update())
        {
            std::vector<uint8_t> outputBuffer_ = listener.getLastFrame();
            if (outputBuffer_.size() == 4)
            {
                int32_t data = static_cast<int32_t>(
                    static_cast<uint32_t>(outputBuffer_[3] << 24) +
                    static_cast<uint32_t>(outputBuffer_[2] << 16) +
                    static_cast<uint32_t>(outputBuffer_[1] << 8) +
                    static_cast<uint32_t>(outputBuffer_[0]));
                std::cout << "int data" << data << std::endl;
            }
        }
        usleep(100000);
    }
}