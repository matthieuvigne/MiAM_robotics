/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/INA226Driver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>


int main (int argc, char *argv[])
{
    RPi_enablePorts();

    INA226 ina;

    if (!ina.init(&RPI_I2C))
    {
        std::cout << "Failed to init INA226" << std::endl;
        return -1;
    }

    Metronome metronome(0.100 * 1e9);

    while (true)
    {

        std::time_t t = std::time(nullptr);
        char timestamp[100];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
        std::string filename = "ina/log" + std::string(timestamp) + ".hdf5";

        Logger logger;
        logger.start(filename);

        for (int i = 0; i < 2001; i++)
        {

            metronome.wait();
            double const currentTime = metronome.getElapsedTime();

            INA226Reading power = ina.read();
            logger.log("INA226.voltage", currentTime, power.voltage);
            logger.log("INA226.current", currentTime, power.current);
            logger.log("INA226.power", currentTime, power.power);

            std::cout << "\r" << std::setw(10) << std::setfill(' ') << std::setprecision(5) << power.voltage << "V";
            std::cout << " " << std::setw(10) << std::setfill(' ') << std::setprecision(5) << power.current << "A" << std::flush;

        }
    }

    return 0;
}


