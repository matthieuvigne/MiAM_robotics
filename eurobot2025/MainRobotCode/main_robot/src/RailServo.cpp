#include "main_robot/RailServo.h"
#include <thread>
#include <unistd.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>

RailServo::RailServo(STSServoDriver *driver, int const& servoId, int const& gpioId, int const& distance, bool inverted, bool calibrateBottom):
    driver_(driver),
    servoId_(servoId),
    gpio_(gpioId),
    travelDistance_(distance),
    sign_(inverted ? -1 : 1),
    calibrateBottom_(calibrateBottom)
{

}

void RailServo::startCalibration()
{
    isCalibrated_ = false;
    std::thread th(&RailServo::calibration, this);
    th.detach();
}

bool RailServo::isCalibrated()
{
    return isCalibrated_;
}

void RailServo::move(double const& targetPosition)
{
    double const delta = std::min(1.0, std::max(0.0, targetPosition)) - currentPosition_;

    int const nStep = sign_ * static_cast<int>(delta * travelDistance_);
    currentPosition_ = targetPosition;

    driver_->setTargetPosition(servoId_, nStep);
}

bool RailServo::isMoving()
{
    return driver_->isMoving(servoId_);
}

double RailServo::getCurrentPosition()
{
    return currentPosition_;
}


void RailServo::calibration()
{
#ifdef SIMULATION
    isCalibrated_ = true;
    return;
#endif

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
    driver_->setMode(servoId_, STS::Mode::STEP);

    currentPosition_ = (calibrateBottom_ ? 0.0 : 1.0);
    isCalibrated_ = true;
}
