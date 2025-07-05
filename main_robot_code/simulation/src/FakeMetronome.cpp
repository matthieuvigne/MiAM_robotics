/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "FakeMetronome.h"

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

FakeMetronome::FakeMetronome(int period):
    Metronome(period)
{
    currentTime_ = 0.0;
}

void FakeMetronome::tick()
{
    mutex_.lock();
    hasTickBeenSent_ = true;
    mutex_.unlock();
    cv_.notify_one();
}


void FakeMetronome::reset()
{
    mutex_.lock();
    hasReset_ = true;
    mutex_.unlock();
    cv_.notify_one();
}


void FakeMetronome::wait()
{
    std::unique_lock<std::mutex> lck(mutex_);
    if (!hasTickBeenSent_)
    {
        cv_.wait(lck);
    }
        hasTickBeenSent_ = false;
    if (hasReset_)
        currentTime_ = 0.0;
    else
        currentTime_ += nPeriod_ / 1.0e9;
    return;
}

double FakeMetronome::getElapsedTime()
{
    return currentTime_;
}


void FakeMetronome::resetLag()
{
}
