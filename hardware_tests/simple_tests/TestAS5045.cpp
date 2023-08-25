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

    SPIWrapper spi(RPI_SPI_01, 1000000);
    AS5045 encoder(&spi, 2);

    while (!encoder.init())
        usleep(50000);

    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
    std::string filename = "log" + std::string(timestamp) + ".csv";

    Logger logger;
    logger.start(filename);

    Metronome metronome(0.010 * 1e9);

    while (true)
    {
        metronome.wait();
        double const currentTime = metronome.getElapsedTime();
        std::vector<double> const pos = encoder.updatePosition();

        std::cout << "\r" << std::setw(10) << std::setfill(' ') << std::setprecision(3) << pos.at(0);
        std::cout << " " << std::setw(10) << std::setfill(' ') << std::setprecision(3) << pos.at(1) << std::flush;

        logger.log("encoder0", currentTime, pos.at(0));
        logger.log("encoder1", currentTime, pos.at(1));
    }

    return 0;
}


