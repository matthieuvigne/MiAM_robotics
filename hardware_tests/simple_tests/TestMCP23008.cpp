#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/MCP23008Driver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    MCP23008 mcp;

    if (!mcp.init(&RPI_I2C))
    {
        std::cout << "Failed to init MCP23008" << std::endl;
        return -1;
    }

    while (true)
    {
        for (int i = 0; i < 8; i++)
        {
            mcp.setPin(i, true);
            std::cout << "Setting" << i << std::endl;
            usleep(500000);
        }
        for (int i = 0; i < 8; i++)
        {
            mcp.setPin(i, false);
            std::cout << "Clearing" << i << std::endl;
            usleep(500000);
        }

        mcp.setOutputs(255);
        std::cout << "Setting all" << std::endl;
        usleep(500000);

        mcp.setOutputs(0);
        std::cout << "Clearing all" << std::endl;
        usleep(500000);
    }

    return 0;
}


