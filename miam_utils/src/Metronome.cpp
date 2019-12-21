/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Metronome.h"

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

Metronome::Metronome(int period):
    nPeriod_(period)
{
    clock_gettime(CLOCK_MONOTONIC, &startTime_);
    targetTime_ = startTime_;
}

void Metronome::wait()
{
    targetTime_.tv_nsec += nPeriod_;
    while(targetTime_.tv_nsec >= 1e9)
    {
        targetTime_.tv_nsec -= 1e9;
        targetTime_.tv_sec ++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(targetTime_), NULL);
}

double Metronome::getElapsedTime()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return currentTime.tv_sec - startTime_.tv_sec + (currentTime.tv_nsec - startTime_.tv_nsec) / 1e9;
}


void Metronome::resetLag()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    targetTime_ = currentTime;
}
