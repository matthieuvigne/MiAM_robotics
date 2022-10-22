/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/AS5045Driver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    SPIWrapper spi(RPI_SPI_00, 1000000);
    AS5045 encoder(&spi);

    while (!encoder.init())
        usleep(50000);

    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
    std::string filename = "log" + std::string(timestamp) + ".csv";

    Logger logger = Logger(filename, "Encoder test", "Info", "time,position");

    Metronome metronome(0.005 * 1e9);

    while (true)
    {
        metronome.wait();
        double const currentTime = metronome.getElapsedTime();
        double const pos = encoder.updatePosition();

        std::cout << "\r" << std::setw(10) << std::setfill(' ') << std::setprecision(3) << pos << std::flush;

        logger.setData(0, currentTime);
        logger.setData(1, pos);
        logger.writeLine();
    }

    return 0;
}


