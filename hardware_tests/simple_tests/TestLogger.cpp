// Test logger performance
#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>

#include <iostream>


int main (int argc, char *argv[])
{
    Logger logger;
    logger.start("test_logger.miam");

    double dtMax = 0;
    double lastTime = 0;
    Metronome metronome(0.002 * 1e9);
    for (int i = 0; i < 1000000; i++)
    {
        metronome.wait();
        double const currentTime = metronome.getElapsedTime();
        double const dt = currentTime - lastTime;
        lastTime = currentTime;
        if (dt > dtMax)
            dtMax = dt;
        std::cout << currentTime << " " << dt << " " << dtMax << std::endl;

        for (int i = 0; i < 50; i++)
            logger.log(std::to_string(i), currentTime, i * currentTime);
    }
    logger.close();
    return 0;
}