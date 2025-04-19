#include "main_robot/RailServo.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/MathUtils.h>

#include <thread>
#include <unistd.h>
#include <algorithm>

#include <iostream>

double const POS_TOLERANCE = 0.02;

RailServo::RailServo(int const& servoId, int const& gpioId, int const& distance, bool inverted, bool calibrateBottom):
    servoId_(servoId),
    gpio_(gpioId),
    travelDistance_(distance),
    sign_(inverted ? -1 : 1),
    calibrateBottom_(calibrateBottom)
{

}

void RailServo::init(STSServoDriver *driver)
{
    driver_ = driver;
}

void RailServo::move(double const& targetPosition)
{
    targetPosition_ = targetPosition;
    currentState_ = RailState::MOVING;
}

void RailServo::abort()
{
    targetPosition_ = std::min(0.95, currentPosition_ + 0.2);
    currentState_ = RailState::MOTION_FAILED;
}

double RailServo::getCurrentPosition() const
{
    return currentPosition_;
}


void RailServo::calibration()
{
    int sign = (calibrateBottom_ ? -1 : 1) * sign_;

    RPi_setupGPIO(gpio_, PI_GPIO_INPUT_PULLUP);

    driver_->setMode(servoId_, STS::Mode::VELOCITY);
    usleep(10000);
    driver_->setTargetVelocity(servoId_, sign * 2500);

    // Hit fast, then slow
    while (RPi_readGPIO(gpio_) == 1)
        usleep(5000);
    driver_->disable(servoId_); // Disable for hard break
    usleep(50000);
    driver_->setTargetVelocity(servoId_, -sign * 3000);
    while (RPi_readGPIO(gpio_) == 0)
        usleep(5000);
    usleep(250000);
    driver_->disable(servoId_);
    usleep(50000);
    driver_->setTargetVelocity(servoId_, sign * 1000);
    while (RPi_readGPIO(gpio_) == 1)
        usleep(5000);

    driver_->disable(servoId_);
    usleep(50000);
    driver_->setMode(servoId_, STS::Mode::VELOCITY);

    currentPosition_ = (calibrateBottom_ ? 0.0 : 1.0);
    targetPosition_ = currentPosition_;
    lastReadPosition_ = driver_->getCurrentPosition(servoId_);

    currentState_ = RailState::TARGET_REACHED;
}

void RailServo::tick()
{
    // Increment position
    int const pos = driver_->getCurrentPosition(servoId_);
    int posDelta = unwrap(lastReadPosition_, pos, 4096) - lastReadPosition_;
    lastReadPosition_ = pos;
    currentPosition_ += sign_ * posDelta / travelDistance_;

    // Motion state machine.
    if (std::abs(currentPosition_ - targetPosition_) > POS_TOLERANCE)
    {
        double const KP = 50000.0;
        double const MIN_VEL = 2000;
        double const MAX_VEL = 5000;

        double velocityTarget = - KP * (currentPosition_ - targetPosition_);
        if (velocityTarget > 0)
        {
            velocityTarget = std::clamp(velocityTarget, MIN_VEL, MAX_VEL);
        }
        else
        {
            velocityTarget = std::clamp(velocityTarget, -MAX_VEL, -MIN_VEL);
        }

        driver_->setTargetVelocity(servoId_, sign_ * velocityTarget);
    }
    else
    {
        driver_->disable(servoId_);
        if (currentState_ == RailState::MOVING)
        {
            currentState_ = RailState::TARGET_REACHED;
        }
    }
}


void RailManager::start(std::vector<RailServo*> rails)
{
    rails_ = rails;
    std::thread th(&RailManager::railControlThread, this);
    th.detach();
}

bool RailManager::areCalibrated() const
{
    return calibDone_;
}

void RailManager::abort()
{
    for (auto& r : rails_)
        if (r->currentState_ == RailState::MOVING)
            r->abort();
}

void RailManager::railControlThread()
{
    pthread_setname_np(pthread_self(), "railManager");

#ifdef SIMULATION
    calibDone_ = true;
    return;
#endif

    calibDone_ = false;
    std::vector<std::thread> calibThreads;
    for (auto& r : rails_)
        calibThreads.push_back(std::thread(&RailServo::calibration, r));
    for (auto& th : calibThreads)
        th.join();
    calibDone_ = true;

    while (true)
    {
        for (auto& r : rails_)
            r->tick();
        usleep(10000);
    }
}