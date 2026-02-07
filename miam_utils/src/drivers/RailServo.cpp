#include <unistd.h>
#include "miam_utils/RailServo.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/MathUtils.h>

#include <algorithm>

double const POS_TOLERANCE = 0.02;

RailServo::RailServo(STSServoDriver *driver, int const& servoId, int const& gpioId, int const& distance, bool inverted, bool calibrateBottom):
    driver_(driver),
    servoId_(servoId),
    gpio_(gpioId),
    travelDistance_(distance),
    sign_(inverted ? -1 : 1),
    calibrateBottom_(calibrateBottom)
{

}

void RailServo::move(double const& targetPosition)
{
    targetPosition_ = targetPosition;
#ifndef SIMULATION
    currentState_ = RailState::MOVING;
#endif
}


double RailServo::getCurrentPosition() const
{
    return currentPosition_;
}


void RailServo::tick()
{
    // Handle calibration
    if (currentState_ == RailState::INIT)
        return;
    if (currentState_ == RailState::CALIBRATING)
    {
        if (calibStep_ == -1)
        {
            RPi_setupGPIO(gpio_, PI_GPIO_INPUT_PULLUP);
            calibStep_ = 0;
        }
        int const sign = (calibrateBottom_ ? -1 : 1) * sign_;

        if (calibStep_ == 0)
        {
            if (RPi_readGPIO(gpio_) == 0)
            {
                driver_->disable(servoId_); // Disable for hard break
                calibStep_ = 1;
                return;
            }
            else
            {
                driver_->setMode(servoId_, STS::Mode::VELOCITY);
                driver_->setTargetVelocity(servoId_, sign * 2500);
            }
        }
        else if (calibStep_ < 100)
        {
            if (RPi_readGPIO(gpio_) == 1)
            {
                calibStep_ ++;
            }
            driver_->setMode(servoId_, STS::Mode::VELOCITY);
            driver_->setTargetVelocity(servoId_, -sign * 3000);
        }
        else
        {
            if (RPi_readGPIO(gpio_) == 0)
            {
                driver_->disable(servoId_); // Disable for hard break
                currentPosition_ = (calibrateBottom_ ? 0.0 : 1.0);
                targetPosition_ = currentPosition_;
                lastReadPosition_ = driver_->getCurrentPosition(servoId_);

                currentState_ = RailState::TARGET_REACHED;
                return;
            }
            driver_->setTargetVelocity(servoId_, sign * 1000);
        }
        return;
    }

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
        double const MAX_VEL = 7000;

        double velocityTarget = - KP * (currentPosition_ - targetPosition_);
        if (velocityTarget > 0)
        {
            velocityTarget = std::clamp(velocityTarget, MIN_VEL, MAX_VEL);
        }
        else
        {
            velocityTarget = std::clamp(velocityTarget, -MAX_VEL, -MIN_VEL);
        }

        driver_->setMode(servoId_, STS::Mode::VELOCITY);
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