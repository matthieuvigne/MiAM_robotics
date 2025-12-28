/// A simple test for the AS5045 encoder

#include <miam_utils/Metronome.h>
#include <miam_utils/Logger.h>
#include <miam_utils/STSServoManager.h>
#include <miam_utils/drivers/STSServoDriver.h>

#include <iostream>
#include <unistd.h>



int main (int argc, char *argv[])
{
    STSServoManager driver;

    if (!driver.init("/dev/ttyAMA0", -1))
    {
        std::cout << "Init failed" << std::endl;
        return -1;
    }


    Logger logger;
    logger.start("logs/test_ServoManager.miam");

    std::vector<unsigned char> servoIds({7, 16, 17, 18, 100});
    for (auto i : servoIds)
        driver.setMode(i, STS::Mode::POSITION);
    driver.setMode(7, STS::Mode::VELOCITY);

    Metronome metronome(5000000);
    double time = 0;
    double lastTime = time;
    while(time < 5.0)
    {
        metronome.wait();
        time = metronome.getElapsedTime();
        int target = 2048 + 200 * std::sin(M_PI * time);

        for (auto i : servoIds)
        {
            double t0 = metronome.getElapsedTime();
            int16_t pos = (i == 7 ? driver.getCurrentVelocity(i) : driver.getCurrentPosition(i));
            double getTime = metronome.getElapsedTime();
            if (i == 7)
                driver.setTargetVelocity(i, (i == 100 ? target / 4: target));
            else
                driver.setTargetPosition(i, (i == 100 ? target / 4: target));
            double setTime = metronome.getElapsedTime();

            // Log
            logger.log("target" + std::to_string(i), time, target);
            logger.log("pos" + std::to_string(i), time, pos);
            logger.log("getDuration" + std::to_string(i), time, getTime - t0);
            logger.log("setDuration" + std::to_string(i), time, setTime - getTime);
        }
        logger.log("readFailed", time, driver.getNReadFailed());
        logger.log("timeDiff", time, time - lastTime);
        lastTime = time;
    }
    return 0;
}


