// Test logger performance
#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>


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
            logger.log(std::to_string(i), 42.0 + 0.1 * i, 2 * currentTime);
    }
    logger.close();
    return 0;
}